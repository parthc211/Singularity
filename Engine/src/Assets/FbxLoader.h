#pragma once
// ---------------------------------------------------------------------------
// FBX front-end: a thin adapter over the vendored ufbx library
// (Engine/third_party/ufbx — single-file C, MIT/Unlicense, zero dependencies)
// feeding the SAME SkeletalMeshData intermediate as the hand-written glTF
// loader, so everything downstream (animation sampling, blending, palettes,
// GPU skinning, the demo scene) is shared. Per the project's rules ufbx is
// "boring plumbing" like ImGui: FBX is a proprietary format whose parsing is
// file-format archaeology, not graphics — the showcase systems stay ours.
//
// Conversion is delegated to ufbx at load time (target axes = left-handed
// +Y-up, meters, Z-mirror, reversed winding to the engine's clockwise front
// faces), so the adapter maps structures instead of doing math:
//   mesh      -> triangulated, per-corner vertices (no dedup), UV V flipped
//                (FBX uses a bottom-left origin, D3D top-left)
//   skin      -> clusters -> parent-before-child Skeleton (same pseudo-root
//                scene-prefix trick as the glTF loader), geometry_to_bone as
//                the inverse bind matrices
//   anim      -> each anim stack baked at a fixed 30 Hz into AnimationClip
//                keys (constant channels collapse to a single key)
//
// Not imported (logged where relevant): materials/textures, blend shapes,
// cameras/lights.
// ---------------------------------------------------------------------------
#include "Assets/GltfLoader.h"   // SkeletalMeshData

namespace SGE {

// Loads any FBX (binary or ASCII, all versions ufbx supports — including
// Mixamo exports dropped into Assets/). Returns false (and logs) on failure.
bool LoadFBX(const char* path, SkeletalMeshData& out);

} // namespace SGE
