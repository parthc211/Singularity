#pragma once
// ---------------------------------------------------------------------------
// Skeleton — a flat joint hierarchy for skinned animation, built on the
// engine's own SIMD math. No scene-graph objects: joints are indices into
// parallel arrays, and the loader guarantees PARENT-BEFORE-CHILD order, so
// local->global pose propagation is one linear pass with no recursion.
//
// Spaces (row-vector convention, like the rest of the engine):
//   local  pose:  joint relative to its parent joint (TRS)
//   global pose:  joint -> model space   global[i] = local[i] * global[parent]
//   InverseBind:  model space (bind pose) -> joint space; the skinning palette
//                 is  palette[i] = InverseBind[i] * global[i], so a bind-pose
//                 vertex goes model -> joint -> animated model in one matrix.
// ---------------------------------------------------------------------------
#include "Math/SimdMath.h"

#include <cstdint>
#include <string>
#include <vector>

namespace SGE::Anim {

constexpr uint32_t kInvalidJoint = 0xFFFFFFFFu;

// One joint's local transform. Deliberately TRS, not a matrix: rotations must
// stay quaternions until after interpolation/blending (matrices don't slerp).
struct JointPose {
    Math::Vec4 T { 0.0f, 0.0f, 0.0f, 0.0f };
    Math::Quat R {};                          // identity
    Math::Vec4 S { 1.0f, 1.0f, 1.0f, 0.0f };
};

struct Skeleton {
    std::vector<std::string> Names;        // debug / clip retargeting by name
    std::vector<uint32_t>    Parents;      // kInvalidJoint for roots; Parents[i] < i
    std::vector<JointPose>   RestPose;     // local; fallback for unkeyed channels
    std::vector<Math::Mat4>  InverseBind;  // model (bind) -> joint space

    uint32_t JointCount() const { return static_cast<uint32_t>(Parents.size()); }
};

} // namespace SGE::Anim
