#include "Assets/FbxLoader.h"
#include "Core/Logger.h"

#include <ufbx.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <format>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace SGE {

namespace {

constexpr double kBakeHz = 30.0;   // animation resample rate

// ufbx_matrix is a 4x3 affine matrix in column-vector convention; its columns
// are exactly the rows of the engine's row-vector Mat4 (transpose-free).
Math::Mat4 FromUfbx(const ufbx_matrix& m)
{
    Math::Mat4 r;
    r.r[0] = _mm_set_ps(0.0f, float(m.cols[0].z), float(m.cols[0].y), float(m.cols[0].x));
    r.r[1] = _mm_set_ps(0.0f, float(m.cols[1].z), float(m.cols[1].y), float(m.cols[1].x));
    r.r[2] = _mm_set_ps(0.0f, float(m.cols[2].z), float(m.cols[2].y), float(m.cols[2].x));
    r.r[3] = _mm_set_ps(1.0f, float(m.cols[3].z), float(m.cols[3].y), float(m.cols[3].x));
    return r;
}

Anim::JointPose FromUfbx(const ufbx_transform& t)
{
    Anim::JointPose p;
    p.T = Math::Vec4(float(t.translation.x), float(t.translation.y), float(t.translation.z), 0.0f);
    p.R = Math::Normalize(Math::Quat(float(t.rotation.x), float(t.rotation.y),
                                     float(t.rotation.z), float(t.rotation.w)));
    p.S = Math::Vec4(float(t.scale.x), float(t.scale.y), float(t.scale.z), 0.0f);
    return p;
}

Math::Mat4 PoseMatrix(const Anim::JointPose& p)
{
    Math::Mat4 m = Math::ToMatrix(p.R);
    m.r[0] = _mm_mul_ps(m.r[0], _mm_set1_ps(p.S.x()));
    m.r[1] = _mm_mul_ps(m.r[1], _mm_set1_ps(p.S.y()));
    m.r[2] = _mm_mul_ps(m.r[2], _mm_set1_ps(p.S.z()));
    m.r[3] = _mm_set_ps(1.0f, p.T.z(), p.T.y(), p.T.x());
    return m;
}

// Local pose of a node relative to a reference world matrix, from ufbx's
// authoritative world transforms: local = refWorld^-1 * nodeWorld (column-
// vector convention). This is deliberately NOT built from local TRS chains —
// ufbx parks axis/unit adjustments in per-node fields outside local_transform,
// and world matrices are the only place everything is guaranteed composed.
Anim::JointPose RelativePose(const ufbx_matrix& refWorld, const ufbx_matrix& nodeWorld)
{
    const ufbx_matrix inv   = ufbx_matrix_invert(&refWorld);
    const ufbx_matrix local = ufbx_matrix_mul(&inv, &nodeWorld);
    return FromUfbx(ufbx_matrix_to_transform(&local));
}

bool NearIdentity(const Math::Mat4& m)
{
    const Math::Mat4 I = Math::Mat4::Identity();
    float fm[16], fi[16];
    for (int r = 0; r < 4; ++r) {
        _mm_storeu_ps(fm + r * 4, m.r[r]);
        _mm_storeu_ps(fi + r * 4, I.r[r]);
    }
    for (int k = 0; k < 16; ++k)
        if (std::fabs(fm[k] - fi[k]) > 1e-5f) return false;
    return true;
}

void FallbackTangent(const float n[3], float out[4])
{
    const bool useX = std::fabs(n[0]) < 0.9f;
    const float ax = useX ? 1.0f : 0.0f, ay = useX ? 0.0f : 1.0f;
    float tx = -n[2] * ay;
    float ty =  n[2] * ax;
    float tz =  n[0] * ay - n[1] * ax;
    const float len = std::sqrt(tx * tx + ty * ty + tz * tz);
    if (len > 1e-8f) { tx /= len; ty /= len; tz /= len; } else { tx = 1; ty = tz = 0; }
    out[0] = tx; out[1] = ty; out[2] = tz; out[3] = 1.0f;
}

// Collapse a channel whose baked keys are all identical to a single key
// (most joints keep constant translation/scale — 3x memory for nothing).
template <typename Channel, typename EqualFn>
void CollapseConstant(Channel& c, EqualFn&& equal)
{
    if (c.Times.size() < 2) return;
    for (size_t i = 1; i < c.Values.size(); ++i)
        if (!equal(c.Values[0], c.Values[i])) return;
    c.Times.resize(1);
    c.Values.resize(1);
}

bool NearVec(Math::Vec4 a, Math::Vec4 b)
{
    return Math::LengthSq(a + b * -1.0f) < 1e-12f;
}
bool NearQuat(Math::Quat a, Math::Quat b)
{
    return std::fabs(Math::Dot(a, b)) > 1.0f - 1e-7f;
}

} // namespace

bool LoadFBX(const char* path, SkeletalMeshData& out)
{
    out = {};

    // ufbx does the coordinate work at load time: left-handed +Y-up meters via
    // a Z-mirror. reverse_winding = true because a mirror alone leaves front
    // faces visually CCW (the same two-flip trap the glTF loader documents);
    // the rewind restores the engine's clockwise-front convention.
    // MODIFY_GEOMETRY bakes the axis/unit conversion into the vertex data
    // itself (ufbx keeps the skin/bind matrices consistent), so imported
    // positions are directly in engine space — and the mirror then really
    // reverses triangle orientation, which reverse_winding corrects back to
    // the engine's clockwise front faces.
    // ADJUST_TRANSFORMS keeps vertex data in geometry space and parks the
    // axis/unit conversion in node transforms — the matrix-relative skeleton
    // reconstruction below picks those up via node_to_world, so the palette is
    // exact. reverse_winding stays false: measured empirically (skinned signed
    // volume matches cube.obj's engine convention with the mirror expressed in
    // transforms), see gltf_test's winding checks.
    ufbx_load_opts opts = {};
    opts.target_axes                 = ufbx_axes_left_handed_y_up;
    opts.target_unit_meters          = 1.0f;
    opts.handedness_conversion_axis  = UFBX_MIRROR_AXIS_Z;
    opts.space_conversion            = UFBX_SPACE_CONVERSION_ADJUST_TRANSFORMS;
    opts.generate_missing_normals    = true;

    ufbx_error error;
    ufbx_scene* scene = ufbx_load_file(path, &opts, &error);
    if (!scene) {
        LogError(std::format("LoadFBX: '{}' — {}", path, error.description.data));
        return false;
    }

    // --- pick the mesh: first skinned one, else first ---
    ufbx_mesh* mesh = nullptr;
    for (size_t i = 0; i < scene->meshes.count && !mesh; ++i)
        if (scene->meshes.data[i]->skin_deformers.count > 0)
            mesh = scene->meshes.data[i];
    if (!mesh && scene->meshes.count > 0)
        mesh = scene->meshes.data[0];
    if (!mesh) {
        LogError(std::format("LoadFBX: '{}' has no mesh", path));
        ufbx_free_scene(scene);
        return false;
    }
    ufbx_skin_deformer* skin = mesh->skin_deformers.count > 0
                             ? mesh->skin_deformers.data[0] : nullptr;

    // --- skeleton from the skin clusters (cluster order = skin joint order) ---
    std::vector<ufbx_node*>                  jointNodes;
    std::unordered_map<const ufbx_node*, int> nodeToJoint;   // -> final skeleton index
    std::vector<uint32_t>                    jointRemap;     // cluster index -> skeleton index
    std::vector<const ufbx_node*>            jointOrigNode;  // by skeleton index; null = pseudo-root
    std::vector<const ufbx_node*>            jointParentNode;// nearest joint ancestor; null for roots

    if (skin) {
        for (size_t c = 0; c < skin->clusters.count; ++c) {
            ufbx_node* bone = skin->clusters.data[c]->bone_node;
            if (!bone) {
                LogError("LoadFBX: skin cluster without a bone node");
                ufbx_free_scene(scene);
                return false;
            }
            jointNodes.push_back(bone);
        }

        // Parent-before-child order via node depth (same scheme as the glTF loader).
        auto depthOf = [](const ufbx_node* n) {
            int d = 0;
            for (const ufbx_node* p = n->parent; p; p = p->parent) ++d;
            return d;
        };
        std::vector<uint32_t> order(jointNodes.size());
        std::iota(order.begin(), order.end(), 0u);
        std::stable_sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
            return depthOf(jointNodes[a]) < depthOf(jointNodes[b]);
        });

        // Scene prefix: everything above the root joint (including ufbx's
        // axis/unit adjustments on ancestor nodes) is the root joint's
        // parent's WORLD transform. Non-identity prefixes become a fixed
        // pseudo-root joint so ComputeGlobals reproduces node_to_world.
        const ufbx_node* prefixNode  = jointNodes[order[0]]->parent;
        const ufbx_matrix prefixWorld = prefixNode ? prefixNode->node_to_world
                                                   : ufbx_identity_matrix;
        const bool     hasPrefix = !NearIdentity(FromUfbx(prefixWorld));
        const uint32_t extra     = hasPrefix ? 1u : 0u;
        const uint32_t total     = uint32_t(jointNodes.size()) + extra;
        if (total > 256) {
            LogError(std::format("LoadFBX: {} joints exceeds the 256-joint palette", total));
            ufbx_free_scene(scene);
            return false;
        }

        jointRemap.resize(jointNodes.size());
        for (uint32_t newIdx = 0; newIdx < jointNodes.size(); ++newIdx) {
            jointRemap[order[newIdx]] = newIdx + extra;
            nodeToJoint[jointNodes[order[newIdx]]] = int(newIdx + extra);
        }

        Anim::Skeleton& sk = out.Skeleton;
        sk.Names.resize(total);
        sk.Parents.resize(total);
        sk.RestPose.resize(total);
        sk.InverseBind.resize(total, Math::Mat4::Identity());
        jointOrigNode.assign(total, nullptr);
        jointParentNode.assign(total, nullptr);

        if (hasPrefix) {
            sk.Names[0]    = "<scene-prefix>";
            sk.Parents[0]  = Anim::kInvalidJoint;
            sk.RestPose[0] = FromUfbx(ufbx_matrix_to_transform(&prefixWorld));
        }

        // Rest pose of each joint = its world transform relative to its parent
        // joint's world (or to the prefix for roots). By induction, composing
        // these in ComputeGlobals reproduces node_to_world exactly, which is
        // what geometry_to_bone (the inverse binds) pairs with.
        for (uint32_t newIdx = 0; newIdx < jointNodes.size(); ++newIdx) {
            const uint32_t   dst  = newIdx + extra;
            const ufbx_node* node = jointNodes[order[newIdx]];

            sk.Names[dst] = node->name.length ? std::string(node->name.data)
                                              : std::format("joint{}", dst);

            const ufbx_node* p = node->parent;
            while (p && !nodeToJoint.count(p)) p = p->parent;
            sk.Parents[dst] = p         ? uint32_t(nodeToJoint[p])
                            : hasPrefix ? 0u
                                        : Anim::kInvalidJoint;
            const ufbx_matrix refWorld = p ? p->node_to_world : prefixWorld;
            sk.RestPose[dst] = RelativePose(refWorld, node->node_to_world);

            sk.InverseBind[dst]  = FromUfbx(skin->clusters.data[order[newIdx]]->geometry_to_bone);
            jointOrigNode[dst]   = node;
            jointParentNode[dst] = p;
        }
    } else {
        // Static fallback: one identity joint (matches the glTF loader).
        out.Skeleton.Names       = { "root" };
        out.Skeleton.Parents     = { Anim::kInvalidJoint };
        out.Skeleton.RestPose.resize(1);
        out.Skeleton.InverseBind = { Math::Mat4::Identity() };
    }

    // --- geometry: triangulate, one vertex per triangle corner ---
    const bool bakeWorld = !skin;   // unskinned: bake the node's world transform
    const Math::Mat4 geoToWorld = FromUfbx(mesh->instances.count > 0
                                               ? mesh->instances.data[0]->geometry_to_world
                                               : ufbx_identity_matrix);

    std::vector<uint32_t> tri(size_t(mesh->max_face_triangles) * 3);
    for (size_t f = 0; f < mesh->faces.count; ++f) {
        const ufbx_face face = mesh->faces.data[f];
        const uint32_t numTris = ufbx_triangulate_face(tri.data(), tri.size(), mesh, face);

        for (uint32_t i = 0; i < numTris * 3; ++i) {
            const uint32_t ix = tri[i];
            SkinnedVertex v = {};

            ufbx_vec3 pos = mesh->vertex_position.values.data[mesh->vertex_position.indices.data[ix]];
            v.position[0] = float(pos.x); v.position[1] = float(pos.y); v.position[2] = float(pos.z);

            if (mesh->vertex_normal.exists) {
                const ufbx_vec3 n = mesh->vertex_normal.values.data[mesh->vertex_normal.indices.data[ix]];
                v.normal[0] = float(n.x); v.normal[1] = float(n.y); v.normal[2] = float(n.z);
            } else {
                v.normal[1] = 1.0f;
            }

            if (mesh->vertex_uv.exists) {
                const ufbx_vec2 uv = mesh->vertex_uv.values.data[mesh->vertex_uv.indices.data[ix]];
                v.texCoord[0] = float(uv.x);
                v.texCoord[1] = 1.0f - float(uv.y);   // FBX V origin is bottom-left; D3D top-left
            }

            FallbackTangent(v.normal, v.tangent);

            if (skin) {
                uint32_t vtx = mesh->vertex_indices.data[ix];
                if (vtx >= skin->vertices.count) vtx = 0;
                const ufbx_skin_vertex sv = skin->vertices.data[vtx];
                float wsum = 0.0f;
                const uint32_t count = sv.num_weights < 4 ? sv.num_weights : 4;
                for (uint32_t k = 0; k < count; ++k) {
                    const ufbx_skin_weight w = skin->weights.data[sv.weight_begin + k];
                    v.joints[k]  = w.cluster_index < jointRemap.size() ? jointRemap[w.cluster_index] : 0;
                    v.weights[k] = float(w.weight);
                    wsum += float(w.weight);
                }
                if (wsum > 1e-6f) {
                    for (int k = 0; k < 4; ++k) v.weights[k] /= wsum;
                } else {
                    v.weights[0] = 1.0f; v.weights[1] = v.weights[2] = v.weights[3] = 0.0f;
                }
            } else {
                v.weights[0] = 1.0f;
                if (bakeWorld) {
                    const Math::Vec4 wp = Math::Transform(
                        Math::Vec4(v.position[0], v.position[1], v.position[2], 1.0f), geoToWorld);
                    v.position[0] = wp.x(); v.position[1] = wp.y(); v.position[2] = wp.z();
                    const Math::Vec4 wn = Math::Normalize3(Math::Transform(
                        Math::Vec4(v.normal[0], v.normal[1], v.normal[2], 0.0f), geoToWorld));
                    v.normal[0] = wn.x(); v.normal[1] = wn.y(); v.normal[2] = wn.z();
                }
            }

            out.Indices.push_back(uint32_t(out.Vertices.size()));
            out.Vertices.push_back(v);
        }
    }

    // --- animations: bake each stack at kBakeHz ---
    for (size_t s = 0; s < scene->anim_stacks.count; ++s) {
        ufbx_anim_stack* stack = scene->anim_stacks.data[s];
        const double duration = stack->time_end - stack->time_begin;
        if (duration <= 1e-6) continue;

        Anim::AnimationClip clip;
        clip.Name     = stack->name.length ? std::string(stack->name.data)
                                           : std::format("take{}", s);
        clip.Duration = float(duration);
        clip.Tracks.resize(out.Skeleton.JointCount());

        // Evaluate the whole scene once per frame and take each joint's world
        // matrix relative to its parent joint's — the same matrix-relative
        // scheme as the rest pose, so bind and animation stay consistent.
        const size_t frames = size_t(std::ceil(duration * kBakeHz)) + 1;
        bool evalOk = true;
        for (size_t fr = 0; fr < frames && evalOk; ++fr) {
            const double t = std::min(double(fr) / kBakeHz, duration);
            ufbx_error evErr;
            ufbx_scene* ev = ufbx_evaluate_scene(scene, stack->anim,
                                                 stack->time_begin + t, nullptr, &evErr);
            if (!ev) {
                LogWarn(std::format("LoadFBX: evaluate failed for clip '{}'", clip.Name));
                evalOk = false;
                break;
            }
            for (uint32_t j = 0; j < out.Skeleton.JointCount(); ++j) {
                const ufbx_node* node = jointOrigNode[j];
                if (!node) continue;   // pseudo-root stays fixed
                const ufbx_node* evNode = ev->nodes.data[node->typed_id];
                const ufbx_node* pj     = jointParentNode[j];
                const ufbx_matrix ref =
                    pj           ? ev->nodes.data[pj->typed_id]->node_to_world
                  : node->parent ? ev->nodes.data[node->parent->typed_id]->node_to_world
                                 : ufbx_identity_matrix;
                const Anim::JointPose p = RelativePose(ref, evNode->node_to_world);

                Anim::JointTrack& track = clip.Tracks[j];
                track.Translation.Times.push_back(float(t));
                track.Translation.Values.push_back(p.T);
                track.Rotation.Times.push_back(float(t));
                track.Rotation.Values.push_back(p.R);
                track.Scale.Times.push_back(float(t));
                track.Scale.Values.push_back(p.S);
            }
            ufbx_free_scene(ev);
        }
        if (!evalOk) continue;

        for (Anim::JointTrack& track : clip.Tracks) {
            CollapseConstant(track.Translation, NearVec);
            CollapseConstant(track.Rotation,    NearQuat);
            CollapseConstant(track.Scale,       NearVec);
        }
        out.Clips.push_back(std::move(clip));
    }

    LogInfo(std::format("LoadFBX: '{}' — {} verts, {} tris, {} joints, {} clip(s)",
                        path, out.Vertices.size(), out.Indices.size() / 3,
                        out.Skeleton.JointCount(), out.Clips.size()));
    ufbx_free_scene(scene);
    return true;
}

} // namespace SGE
