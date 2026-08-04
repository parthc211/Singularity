#pragma once
// ---------------------------------------------------------------------------
// Hand-written glTF 2.0 loader for skinned characters.
//
// Supports: .glb (binary container) and .gltf with external or base64
// data-URI buffers; accessors over interleaved or tight buffer views with all
// component types incl. normalized integers; triangle primitives (indexed or
// not); skins (inverse bind matrices, <= 256 joints); animations with
// translation/rotation/scale channels (LINEAR native; STEP and CUBICSPLINE
// are approximated as linear and logged).
//
// Not supported (logged when encountered): sparse accessors, morph targets,
// non-triangle primitive modes, multiple skins (first one wins).
//
// Conventions: glTF is right-handed +Y-up with column-major matrices; the
// engine is left-handed +Y-up row-vector. Conversion happens at import:
//   positions/normals/tangent.xyz/translations:  z -> -z
//   quaternions:                                 (x,y,z,w) -> (-x,-y,z,w)
//   matrices:                                    C * M * C  (C = diag(1,1,-1,1))
//   tangent handedness w:                        flipped (mirror flips it)
//   triangle winding:                            unchanged — the Z-mirror
//     already turns glTF's CCW front faces into the engine's CW front faces.
//   UVs: unchanged (glTF's top-left origin matches D3D).
//
// Joints are re-ordered parent-before-child (glTF does not guarantee it) and
// JOINTS_0 indices, inverse binds, and animation targets are remapped to match
// — Anim::ComputeGlobals relies on that order.
// ---------------------------------------------------------------------------
#include "Animation/AnimationClip.h"
#include "Animation/Skeleton.h"

#include <cstdint>
#include <vector>

namespace SGE {

// CPU-side skinned vertex. Field order follows the same prefix contract as
// MeshVertex (input layouts are reflected with APPEND offsets): POSITION,
// NORMAL, TEXCOORD, TANGENT, then BLENDINDICES / BLENDWEIGHT.
//
// joints are uint32 (not the u8 the file stores) because the reflected input
// layout maps HLSL uint4 to R32G32B32A32_UINT — the vertex must match that.
struct SkinnedVertex {
    float    position[3];
    float    normal[3];
    float    texCoord[2];
    float    tangent[4];  // xyz + handedness w (engine convention, post-mirror)
    uint32_t joints[4];   // skeleton joint indices (parent-before-child order)
    float    weights[4];  // normalized to sum 1
};

// Everything one glTF character yields, renderer-agnostic. A second importer
// (e.g. ufbx) would target this same struct.
struct SkeletalMeshData {
    std::vector<SkinnedVertex>       Vertices;   // model space (bind pose)
    std::vector<uint32_t>            Indices;
    Anim::Skeleton                   Skeleton;
    std::vector<Anim::AnimationClip> Clips;
};

// Loads a .glb or .gltf file. Merges all primitives of the first skinned
// node's mesh; if the file has no skin, imports the first mesh statically
// (baked node transform, one identity joint, full weight on it) so it still
// renders through the skinned path. Returns false (and logs) on failure.
bool LoadGLTF(const char* path, SkeletalMeshData& out);

} // namespace SGE
