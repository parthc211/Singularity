// Headless verification of the asset loaders (hand-written glTF + the ufbx
// FBX front-end) against the repo's sample assets, plus a synthetic glTF with
// hand-derivable ground truth (sparse accessors, CUBICSPLINE, STEP).
// Run from the Sandbox directory (ctest sets that), so asset paths match the
// app's own ("Assets/...").
#include "Assets/GltfLoader.h"
#include "Assets/FbxLoader.h"
#include "Animation/Animation.h"
#include "Renderer/MeshLoader.h"   // cube.obj = the engine's winding reference

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace SGE;
using namespace SGE::Math;
using namespace SGE::Anim;

static int g_failures = 0;

static void Check(const char* name, bool ok)
{
    std::printf("%-62s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

static bool Near(float a, float b, float eps) { return std::fabs(a - b) < eps; }

// Max |palette[i] - palette[0]| over all joints at the rest pose. Rest == bind
// for these assets, so the rest palette must be one UNIFORM transform: it may
// rigidly re-orient the whole character (scene nodes above the root joint —
// CesiumMan's Z-up fix — legitimately do), but it must not deform the mesh.
static float RestPaletteUniformError(const Skeleton& sk)
{
    std::vector<JointPose> pose = sk.RestPose;
    std::vector<Mat4> globals, palette;
    ComputeGlobals(sk, pose, globals);
    ComputePalette(sk, globals, palette);

    float maxErr = 0.0f;
    for (const Mat4& m : palette) {
        for (int r = 0; r < 4; ++r) {
            float fm[4], f0[4];
            _mm_storeu_ps(fm, m.r[r]);
            _mm_storeu_ps(f0, palette[0].r[r]);
            for (int c = 0; c < 4; ++c)
                maxErr = std::max(maxErr, std::fabs(fm[c] - f0[c]));
        }
    }
    return maxErr;
}

// Signed volume via the divergence theorem: sum of v0 . (v1 x v2) / 6 over
// triangles. For a closed mesh the SIGN encodes the winding orientation, so a
// glTF import wound like cube.obj (the engine's clockwise-front convention,
// proven correct by 14 scenes) must produce the same sign. This is the check
// that catches inside-out imports, which flat shading visually masks.
template <typename V>
static double SignedVolume(const std::vector<V>& verts, const std::vector<uint32_t>& idx)
{
    double vol = 0.0;
    for (size_t i = 0; i + 2 < idx.size(); i += 3) {
        const float* a = verts[idx[i + 0]].position;
        const float* b = verts[idx[i + 1]].position;
        const float* c = verts[idx[i + 2]].position;
        const double cx = double(b[1]) * c[2] - double(b[2]) * c[1];
        const double cy = double(b[2]) * c[0] - double(b[0]) * c[2];
        const double cz = double(b[0]) * c[1] - double(b[1]) * c[0];
        vol += a[0] * cx + a[1] * cy + a[2] * cz;
    }
    return vol / 6.0;
}

// Extents of the rest-pose SKINNED mesh (what the viewer actually sees).
static void RestSkinnedExtents(const SkeletalMeshData& d, float ext[3])
{
    std::vector<JointPose> pose = d.Skeleton.RestPose;
    std::vector<Mat4> globals, palette;
    ComputeGlobals(d.Skeleton, pose, globals);
    ComputePalette(d.Skeleton, globals, palette);

    float lo[3] = { 1e9f, 1e9f, 1e9f }, hi[3] = { -1e9f, -1e9f, -1e9f };
    for (const SkinnedVertex& sv : d.Vertices) {
        Vec4 acc(0, 0, 0, 0);
        for (int k = 0; k < 4; ++k)
            acc += Transform(Vec4(sv.position[0], sv.position[1], sv.position[2], 1.0f),
                             palette[sv.joints[k]]) * sv.weights[k];
        const float p[3] = { acc.x(), acc.y(), acc.z() };
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], p[a]);
            hi[a] = std::max(hi[a], p[a]);
        }
    }
    for (int a = 0; a < 3; ++a) ext[a] = hi[a] - lo[a];
}

// Shared invariants for any loaded character. restIsBind: glTF sample assets
// store the bind pose as the rest pose, making the uniform-palette check
// valid; FBX files legitimately save a DIFFERENT current pose, so that check
// is skipped and a units-sanity check (skinned-vs-raw extents ratio at clip
// start) guards against scale/composition bugs instead.
static void CheckStructural(const char* label, const SkeletalMeshData& d,
                            bool restIsBind = true)
{
    char name[128];

    bool ordered = true;
    for (uint32_t j = 0; j < d.Skeleton.JointCount(); ++j) {
        const uint32_t p = d.Skeleton.Parents[j];
        ordered = ordered && (p == kInvalidJoint || p < j);
    }
    std::snprintf(name, sizeof(name), "%s: parents precede children", label);
    Check(name, ordered);

    bool weightsOk = true, jointsOk = true;
    for (const SkinnedVertex& v : d.Vertices) {
        const float sum = v.weights[0] + v.weights[1] + v.weights[2] + v.weights[3];
        weightsOk = weightsOk && Near(sum, 1.0f, 1e-3f);
        for (int k = 0; k < 4; ++k)
            jointsOk = jointsOk && (v.joints[k] < d.Skeleton.JointCount());
    }
    std::snprintf(name, sizeof(name), "%s: weights sum to 1, joint indices in range", label);
    Check(name, weightsOk && jointsOk);

    if (restIsBind) {
        const float err = RestPaletteUniformError(d.Skeleton);
        std::snprintf(name, sizeof(name), "%s: rest palette uniform rigid transform (err %.2e)", label, err);
        Check(name, err < 1e-2f);
    }

    // Sample every clip at a few phases: all skinned positions must stay finite
    // and inside a sane bound (catches matrix-layout / conversion bugs).
    bool sampledOk = true;
    for (const AnimationClip& clip : d.Clips) {
        for (float phase : { 0.0f, 0.33f, 0.75f, 1.0f }) {
            std::vector<JointPose> pose;
            SampleClip(d.Skeleton, clip, clip.Duration * phase, pose);
            std::vector<Mat4> globals, palette;
            ComputeGlobals(d.Skeleton, pose, globals);
            ComputePalette(d.Skeleton, globals, palette);
            for (size_t v = 0; v < d.Vertices.size(); v += 97) {  // sparse sample
                const SkinnedVertex& sv = d.Vertices[v];
                Vec4 acc(0, 0, 0, 0);
                for (int k = 0; k < 4; ++k)
                    acc += Transform(Vec4(sv.position[0], sv.position[1], sv.position[2], 1.0f),
                                     palette[sv.joints[k]]) * sv.weights[k];
                for (float f : { acc.x(), acc.y(), acc.z() })
                    sampledOk = sampledOk && std::isfinite(f) && std::fabs(f) < 1e4f;
            }
        }
    }
    std::snprintf(name, sizeof(name), "%s: all clips sample to finite, bounded poses", label);
    Check(name, sampledOk);
}

int main()
{
    const char* assets = "Assets/";
    char path[256];

    // ---- SimpleSkin (external .bin buffers): known ground truth ----
    {
        SkeletalMeshData d;
        std::snprintf(path, sizeof(path), "%sSimpleSkin.gltf", assets);
        Check("SimpleSkin: loads", LoadGLTF(path, d));

        Check("SimpleSkin: 10 verts, 24 indices, 2 joints, 1 clip",
              d.Vertices.size() == 10 && d.Indices.size() == 24 &&
              d.Skeleton.JointCount() == 2 && d.Clips.size() == 1);
        Check("SimpleSkin: hierarchy root -> child",
              d.Skeleton.Parents[0] == kInvalidJoint && d.Skeleton.Parents[1] == 0);
        Check("SimpleSkin: clip duration 5.5s",
              !d.Clips.empty() && Near(d.Clips[0].Duration, 5.5f, 1e-4f));
        Check("SimpleSkin: rotation channel on child joint, 12 keys",
              !d.Clips.empty() && d.Clips[0].Tracks.size() == 2 &&
              d.Clips[0].Tracks[1].Rotation.Times.size() == 12 &&
              d.Clips[0].Tracks[0].Rotation.Times.empty());

        // Geometry: a 1x2 strip in the XY plane (z = 0 everywhere).
        bool geomOk = !d.Vertices.empty();
        for (const SkinnedVertex& v : d.Vertices)
            geomOk = geomOk && Near(v.position[2], 0.0f, 1e-6f)
                             && v.position[1] >= -1e-6f && v.position[1] <= 2.0f + 1e-6f;
        Check("SimpleSkin: geometry in expected bounds, z == 0", geomOk);

        // The animation must actually move the top of the strip.
        if (!d.Clips.empty() && d.Skeleton.JointCount() == 2) {
            std::vector<JointPose> pose;
            std::vector<Mat4> globals, palette;

            auto topVertexX = [&](float t) {
                SampleClip(d.Skeleton, d.Clips[0], t, pose);
                ComputeGlobals(d.Skeleton, pose, globals);
                ComputePalette(d.Skeleton, globals, palette);
                // vertex 8/9 are the y=2 top row; take the last vertex.
                const SkinnedVertex& sv = d.Vertices.back();
                Vec4 acc(0, 0, 0, 0);
                for (int k = 0; k < 4; ++k)
                    acc += Transform(Vec4(sv.position[0], sv.position[1], sv.position[2], 1.0f),
                                     palette[sv.joints[k]]) * sv.weights[k];
                return acc.x();
            };
            const float x0 = topVertexX(0.0f);
            float maxDelta = 0.0f;
            for (float t = 0.25f; t <= 5.5f; t += 0.25f)
                maxDelta = std::max(maxDelta, std::fabs(topVertexX(t) - x0));
            Check("SimpleSkin: animation displaces the top vertex", maxDelta > 0.1f);
        }
        CheckStructural("SimpleSkin", d);
    }

    // ---- SimpleSkin embedded variant (base64 data-URI buffers) ----
    {
        SkeletalMeshData d;
        std::snprintf(path, sizeof(path), "%sSimpleSkinEmbedded.gltf", assets);
        const bool ok = LoadGLTF(path, d);
        Check("SimpleSkinEmbedded: loads via base64 data URIs",
              ok && d.Vertices.size() == 10 && d.Skeleton.JointCount() == 2 &&
              !d.Clips.empty() && Near(d.Clips[0].Duration, 5.5f, 1e-4f));
    }

    // ---- CesiumMan.glb (GLB container, real rigged character) ----
    {
        SkeletalMeshData d;
        std::snprintf(path, sizeof(path), "%sCesiumMan.glb", assets);
        Check("CesiumMan: loads (GLB)", LoadGLTF(path, d));
        std::printf("    -> %zu verts, %zu tris, %u joints, %zu clip(s), %.2fs\n",
                    d.Vertices.size(), d.Indices.size() / 3, d.Skeleton.JointCount(),
                    d.Clips.size(), d.Clips.empty() ? 0.0f : d.Clips[0].Duration);
        Check("CesiumMan: has skeleton, geometry and a clip",
              d.Vertices.size() > 100 && d.Indices.size() % 3 == 0 &&
              d.Skeleton.JointCount() >= 2 && !d.Clips.empty() &&
              d.Clips[0].Duration > 0.1f);

        // The regression that shipped him lying down: at rest the SKINNED
        // character must be tallest along +Y (a standing human), which requires
        // the scene-prefix rotation above the root joint to be applied.
        float ext[3];
        RestSkinnedExtents(d, ext);
        std::printf("    -> rest extents x %.2f, y %.2f, z %.2f\n", ext[0], ext[1], ext[2]);
        Check("CesiumMan: stands upright at rest (Y is the major axis)",
              ext[1] > ext[0] && ext[1] > ext[2]);

        // Embedded texture: CesiumMan's skin is a PNG inside the GLB bin chunk.
        std::printf("    -> albedo %ux%u\n", d.BaseColorImage.width, d.BaseColorImage.height);
        bool texOk = d.BaseColorImage.IsValid() &&
                     d.BaseColorImage.width >= 64 && d.BaseColorImage.height >= 64;
        if (texOk) {
            // Not uniformly one color (i.e., actually decoded image content).
            const auto& px = d.BaseColorImage.pixels;
            bool varies = false;
            for (size_t i = 4; i < px.size() && !varies; i += 4)
                varies = px[i] != px[0] || px[i + 1] != px[1] || px[i + 2] != px[2];
            texOk = varies;
        }
        Check("CesiumMan: embedded base-color texture decodes with content", texOk);

        // Winding regression: the import must be wound like cube.obj (the
        // engine convention), or the rasterizer culls the character's front
        // faces and renders him inside-out.
        std::vector<MeshVertex> cubeVerts;
        std::vector<uint32_t>   cubeIdx;
        const bool cubeOk = LoadOBJ("Assets/cube.obj",
                                    cubeVerts, cubeIdx);
        const double cubeVol = cubeOk ? SignedVolume(cubeVerts, cubeIdx) : 0.0;
        const double manVol  = SignedVolume(d.Vertices, d.Indices);
        std::printf("    -> signed volume: cube %.3f, CesiumMan %.3f\n", cubeVol, manVol);
        Check("CesiumMan: winding matches the engine convention (cube.obj sign)",
              cubeOk && std::fabs(cubeVol) > 1e-6 && std::fabs(manVol) > 1e-6 &&
              (cubeVol > 0) == (manVol > 0));
        CheckStructural("CesiumMan", d);
    }

    // ---- Fox.glb (GLB, multiple clips) ----
    {
        SkeletalMeshData d;
        std::snprintf(path, sizeof(path), "%sFox.glb", assets);
        Check("Fox: loads (GLB)", LoadGLTF(path, d));
        std::printf("    -> %zu verts, %zu tris, %u joints, %zu clip(s)\n",
                    d.Vertices.size(), d.Indices.size() / 3,
                    d.Skeleton.JointCount(), d.Clips.size());
        Check("Fox: multiple clips with names",
              d.Clips.size() >= 2 && !d.Clips[0].Name.empty());
        std::printf("    -> albedo %ux%u\n", d.BaseColorImage.width, d.BaseColorImage.height);
        Check("Fox: embedded base-color texture decodes",
              d.BaseColorImage.IsValid() && d.BaseColorImage.width >= 64);
        CheckStructural("Fox", d);
    }

    // ---- Synthetic glTF: sparse accessor + exact CUBICSPLINE/STEP sampling ----
    // Built at runtime (data-URI buffer) so the ground truth is hand-derivable.
    {
        std::vector<uint8_t> buf;
        auto pf  = [&](float f)    { uint8_t p[4]; std::memcpy(p, &f, 4); buf.insert(buf.end(), p, p + 4); };
        auto pu16= [&](uint16_t v) { uint8_t p[2]; std::memcpy(p, &v, 2); buf.insert(buf.end(), p, p + 2); };

        // 0: positions, 3 x vec3
        pf(0); pf(0); pf(0);  pf(1); pf(0); pf(0);  pf(0); pf(1); pf(0);
        // 36: indices u16 x3 (+2 pad)
        pu16(0); pu16(1); pu16(2); pu16(0);
        // 44: JOINTS_0 u16x4 x3 (all joint 0)
        for (int k = 0; k < 12; ++k) pu16(0);
        // 68: WEIGHTS_0 f32x4 x3
        for (int v = 0; v < 3; ++v) { pf(1); pf(0); pf(0); pf(0); }
        // 116: IBMs, 2 x mat4 col-major (identity; identity with col3=(0,-1,0,1))
        const float I[16]  = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
        const float T2[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,-1,0,1};
        for (float f : I)  pf(f);
        for (float f : T2) pf(f);
        // 244: rotation times (0, 1)
        pf(0); pf(1);
        // 252: rotation CUBICSPLINE output: (inTan, value, outTan) x 2 keys
        const float q90 = 0.70710678f;
        for (int k = 0; k < 4; ++k) pf(0);                    // a0
        pf(0); pf(0); pf(0); pf(1);                           // v0 = identity
        for (int k = 0; k < 4; ++k) pf(0);                    // b0
        for (int k = 0; k < 4; ++k) pf(0);                    // a1
        pf(0); pf(0); pf(q90); pf(q90);                       // v1 = 90 deg about Z
        for (int k = 0; k < 4; ++k) pf(0);                    // b1
        // 348: step times (0, 0.5, 1)
        pf(0); pf(0.5f); pf(1);
        // 360: step values, 3 x vec3
        pf(0); pf(1); pf(0);  pf(1); pf(1); pf(0);  pf(2); pf(1); pf(0);
        // 396: sparse index u16 (+2 pad), 400: sparse value vec3 (5,5,5)
        pu16(2); pu16(0);
        pf(5); pf(5); pf(5);

        // base64 for the data URI
        static const char* tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string b64;
        for (size_t i = 0; i < buf.size(); i += 3) {
            uint32_t v = uint32_t(buf[i]) << 16;
            if (i + 1 < buf.size()) v |= uint32_t(buf[i + 1]) << 8;
            if (i + 2 < buf.size()) v |= uint32_t(buf[i + 2]);
            b64 += tbl[(v >> 18) & 63];
            b64 += tbl[(v >> 12) & 63];
            b64 += (i + 1 < buf.size()) ? tbl[(v >> 6) & 63] : '=';
            b64 += (i + 2 < buf.size()) ? tbl[v & 63] : '=';
        }

        std::string json = R"({
"asset":{"version":"2.0"},
"scene":0,"scenes":[{"nodes":[0,1]}],
"nodes":[{"mesh":0,"skin":0},{"children":[2]},{"translation":[0,1,0]}],
"meshes":[{"primitives":[{"attributes":{"POSITION":1,"JOINTS_0":2,"WEIGHTS_0":3},"indices":0}]}],
"skins":[{"inverseBindMatrices":4,"joints":[1,2]}],
"animations":[{"name":"synthetic",
 "channels":[{"sampler":0,"target":{"node":2,"path":"rotation"}},
             {"sampler":1,"target":{"node":2,"path":"translation"}}],
 "samplers":[{"input":5,"interpolation":"CUBICSPLINE","output":6},
             {"input":7,"interpolation":"STEP","output":8}]}],
"bufferViews":[
 {"buffer":0,"byteOffset":0,"byteLength":36},
 {"buffer":0,"byteOffset":36,"byteLength":6},
 {"buffer":0,"byteOffset":44,"byteLength":24},
 {"buffer":0,"byteOffset":68,"byteLength":48},
 {"buffer":0,"byteOffset":116,"byteLength":128},
 {"buffer":0,"byteOffset":244,"byteLength":8},
 {"buffer":0,"byteOffset":252,"byteLength":96},
 {"buffer":0,"byteOffset":348,"byteLength":12},
 {"buffer":0,"byteOffset":360,"byteLength":36},
 {"buffer":0,"byteOffset":396,"byteLength":2},
 {"buffer":0,"byteOffset":400,"byteLength":12}],
"accessors":[
 {"bufferView":1,"componentType":5123,"count":3,"type":"SCALAR"},
 {"bufferView":0,"componentType":5126,"count":3,"type":"VEC3",
  "sparse":{"count":1,"indices":{"bufferView":9,"componentType":5123},"values":{"bufferView":10}}},
 {"bufferView":2,"componentType":5123,"count":3,"type":"VEC4"},
 {"bufferView":3,"componentType":5126,"count":3,"type":"VEC4"},
 {"bufferView":4,"componentType":5126,"count":2,"type":"MAT4"},
 {"bufferView":5,"componentType":5126,"count":2,"type":"SCALAR"},
 {"bufferView":6,"componentType":5126,"count":6,"type":"VEC4"},
 {"bufferView":7,"componentType":5126,"count":3,"type":"SCALAR"},
 {"bufferView":8,"componentType":5126,"count":3,"type":"VEC3"}],
"buffers":[{"byteLength":412,"uri":"data:application/octet-stream;base64,)" + b64 + "\"}]}";

        const std::string synthPath =
            (std::filesystem::temp_directory_path() / "sge_synthetic_test.gltf").string();
        { std::ofstream f(synthPath, std::ios::binary); f << json; }

        SkeletalMeshData d;
        Check("Synthetic: loads (sparse + cubic + step)", LoadGLTF(synthPath.c_str(), d));
        Check("Synthetic: 3 verts, 2 joints, 1 clip",
              d.Vertices.size() == 3 && d.Skeleton.JointCount() == 2 && d.Clips.size() == 1);

        // Sparse accessor: vertex 2's position replaced with (5,5,5) -> engine (5,5,-5).
        Check("Synthetic: sparse substitution applied (z-mirrored)",
              d.Vertices.size() == 3 &&
              Near(d.Vertices[2].position[0], 5, 1e-6f) &&
              Near(d.Vertices[2].position[1], 5, 1e-6f) &&
              Near(d.Vertices[2].position[2], -5, 1e-6f));

        if (!d.Clips.empty() && d.Clips[0].Tracks.size() == 2) {
            const JointTrack& tr = d.Clips[0].Tracks[1];
            Check("Synthetic: channel modes + tangents populated",
                  tr.Rotation.Interp == Interpolation::CubicSpline &&
                  tr.Rotation.InTan.size() == 2 && tr.Rotation.OutTan.size() == 2 &&
                  tr.Translation.Interp == Interpolation::Step);

            // CUBICSPLINE with zero tangents at t=0.25: q = normalize(h00*v0 + h01*v1),
            // h00(0.25) = 0.84375, h01(0.25) = 0.15625 (hand-derived Hermite basis).
            std::vector<JointPose> pose;
            SampleClip(d.Skeleton, d.Clips[0], 0.25f, pose);
            const float ez = 0.15625f * q90;
            const float ew = 0.84375f + 0.15625f * q90;
            const float el = std::sqrt(ez * ez + ew * ew);
            float fq[4]; _mm_storeu_ps(fq, pose[1].R.v);
            Check("Synthetic: CUBICSPLINE rotation exact at t=0.25",
                  Near(fq[0], 0, 1e-6f) && Near(fq[1], 0, 1e-6f) &&
                  Near(fq[2], ez / el, 1e-5f) && Near(fq[3], ew / el, 1e-5f));

            // STEP holds the previous key exactly.
            SampleClip(d.Skeleton, d.Clips[0], 0.49f, pose);
            const bool hold0 = Near(pose[1].T.x(), 0, 1e-6f) &&
                               Near(pose[1].T.y(), 1, 1e-6f) &&
                               Near(pose[1].T.z(), 0, 1e-6f);
            SampleClip(d.Skeleton, d.Clips[0], 0.75f, pose);
            const bool hold1 = Near(pose[1].T.x(), 1, 1e-6f) &&
                               Near(pose[1].T.y(), 1, 1e-6f) &&
                               Near(pose[1].T.z(), 0, 1e-6f);
            Check("Synthetic: STEP holds keys exactly", hold0 && hold1);
        } else {
            Check("Synthetic: clip tracks present", false);
        }
    }

    // ---- FBX front-end (ufbx adapter into the same SkeletalMeshData) ----
    // Winding reference for the FBX path too. FBX conversion keeps the mirror
    // in the transforms, so raw-vertex volume is meaningless — orientation and
    // units must be measured on the SKINNED mesh (what the rasterizer sees).
    std::vector<MeshVertex> cubeVerts;
    std::vector<uint32_t>   cubeIdx;
    LoadOBJ("Assets/cube.obj", cubeVerts, cubeIdx);
    const double cubeVol = SignedVolume(cubeVerts, cubeIdx);

    auto skinnedVolumeAndExtent = [](const SkeletalMeshData& d, double& vol, float& maxExtent) {
        std::vector<JointPose> pose = d.Skeleton.RestPose;
        if (!d.Clips.empty()) SampleClip(d.Skeleton, d.Clips[0], 0.0f, pose);
        std::vector<Mat4> globals, palette;
        ComputeGlobals(d.Skeleton, pose, globals);
        ComputePalette(d.Skeleton, globals, palette);

        std::vector<Vec4> skinned(d.Vertices.size());
        float lo[3] = { 1e9f, 1e9f, 1e9f }, hi[3] = { -1e9f, -1e9f, -1e9f };
        for (size_t i = 0; i < d.Vertices.size(); ++i) {
            const SkinnedVertex& sv = d.Vertices[i];
            Vec4 acc(0, 0, 0, 0);
            for (int k = 0; k < 4; ++k)
                acc += Transform(Vec4(sv.position[0], sv.position[1], sv.position[2], 1.0f),
                                 palette[sv.joints[k]]) * sv.weights[k];
            skinned[i] = acc;
            const float p[3] = { acc.x(), acc.y(), acc.z() };
            for (int a = 0; a < 3; ++a) {
                lo[a] = std::min(lo[a], p[a]);
                hi[a] = std::max(hi[a], p[a]);
            }
        }
        maxExtent = std::max({ hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2] });

        vol = 0.0;
        for (size_t i = 0; i + 2 < d.Indices.size(); i += 3) {
            const Vec4 a = skinned[d.Indices[i]], b = skinned[d.Indices[i + 1]], c = skinned[d.Indices[i + 2]];
            const double cx = double(b.y()) * c.z() - double(b.z()) * c.y();
            const double cy = double(b.z()) * c.x() - double(b.x()) * c.z();
            const double cz = double(b.x()) * c.y() - double(b.y()) * c.x();
            vol += a.x() * cx + a.y() * cy + a.z() * cz;
        }
        vol /= 6.0;
    };

    // maya_game_sausage_wiggle: the canonical skinned + animated ufbx sample.
    {
        SkeletalMeshData d;
        std::snprintf(path, sizeof(path), "%smaya_game_sausage_wiggle.fbx", assets);
        Check("Sausage FBX: loads", LoadFBX(path, d));
        std::printf("    -> %zu verts, %zu tris, %u joints, %zu clip(s), %.2fs\n",
                    d.Vertices.size(), d.Indices.size() / 3, d.Skeleton.JointCount(),
                    d.Clips.size(), d.Clips.empty() ? 0.0f : d.Clips[0].Duration);
        Check("Sausage FBX: skinned mesh with a clip",
              d.Vertices.size() > 10 && d.Skeleton.JointCount() >= 2 &&
              !d.Clips.empty() && d.Clips[0].Duration > 0.1f);
        double vol; float ext;
        skinnedVolumeAndExtent(d, vol, ext);
        std::printf("    -> skinned volume %+.2e, max extent %.3fm (cube vol %.3f)\n", vol, ext, cubeVol);
        Check("Sausage FBX: skinned winding matches the engine convention",
              std::fabs(vol) > 1e-9 && (vol > 0) == (cubeVol > 0));
        Check("Sausage FBX: skinned size in sane meter range",
              ext > 0.005f && ext < 100.0f);
        CheckStructural("Sausage FBX", d, /*restIsBind=*/false);
    }

    // KenneyCharacter: a rigged character (multiple joints, real hierarchy).
    {
        SkeletalMeshData d;
        std::snprintf(path, sizeof(path), "%sKenneyCharacter.fbx", assets);
        Check("Kenney FBX: loads", LoadFBX(path, d));
        std::printf("    -> %zu verts, %zu tris, %u joints, %zu clip(s)\n",
                    d.Vertices.size(), d.Indices.size() / 3,
                    d.Skeleton.JointCount(), d.Clips.size());
        Check("Kenney FBX: skinned character",
              d.Vertices.size() > 100 && d.Skeleton.JointCount() >= 4);
        double vol; float ext;
        skinnedVolumeAndExtent(d, vol, ext);
        std::printf("    -> skinned volume %+.2e, max extent %.3fm\n", vol, ext);
        Check("Kenney FBX: skinned winding matches the engine convention",
              std::fabs(vol) > 1e-9 && (vol > 0) == (cubeVol > 0));
        Check("Kenney FBX: skinned size in sane meter range",
              ext > 0.005f && ext < 100.0f);
        CheckStructural("Kenney FBX", d, /*restIsBind=*/false);
    }

    std::printf("\n%s\n", g_failures == 0 ? "ALL PASS" : "FAILURES");
    return g_failures;
}
