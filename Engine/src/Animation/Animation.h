#pragma once
// ---------------------------------------------------------------------------
// The pose pipeline, as free functions plus a small stateful player:
//
//   SampleClip      clip + time            -> local pose  (lerp T/S, slerp R)
//   BlendPoses      two local poses + t    -> local pose  (nlerp R) — crossfades
//   ComputeGlobals  local pose             -> joint->model matrices (one pass)
//   ComputePalette  globals + inverse bind -> skinning matrices
//
// All functions are pure and ECS/renderer-independent so they test headless
// and can later run per-character inside JobSystem jobs.
//
// AnimationPlayer adds playback state (time, looping) and per-channel key
// cursors: steady playback resumes the key search from the previous frame's
// key instead of binary-searching every channel every frame.
// ---------------------------------------------------------------------------
#include "Animation/AnimationClip.h"
#include "Animation/Skeleton.h"

#include <cstdint>
#include <vector>

namespace SGE::Anim {

// Sample every track at time t (seconds, caller pre-wraps/clamps) into
// outLocal, resized to the skeleton's joint count. Unkeyed channels and
// untracked joints fall back to the skeleton rest pose. cursors is optional
// scratch (3 per joint, auto-resized): pass the same vector across frames for
// O(1) steady-state playback, or nullptr for stateless binary-search sampling.
void SampleClip(const Skeleton& skeleton, const AnimationClip& clip, float t,
                std::vector<JointPose>& outLocal,
                std::vector<uint32_t>* cursors = nullptr);

// out[i] = lerp(a[i], b[i], t) — translations/scales lerp, rotations nlerp.
// out may alias a or b. Sizes must match.
void BlendPoses(const std::vector<JointPose>& a, const std::vector<JointPose>& b,
                float t, std::vector<JointPose>& out);

// Additive layering. MakeAdditive turns a sampled pose into a DELTA relative
// to a reference pose (conventionally the layer clip's first frame):
// translations subtract, rotations become conj(ref) * pose, scales divide.
// ApplyAdditive composes that delta ONTO base in place, scaled by weight and
// an optional per-joint mask (weight_j = weight * mask[j]): translations add,
// rotations compose via slerp(identity -> delta), scales multiply toward the
// delta. Invariant: ApplyAdditive(ref, MakeAdditive(pose, ref), 1) == pose.
// outDelta may alias pose.
void MakeAdditive(const std::vector<JointPose>& pose,
                  const std::vector<JointPose>& reference,
                  std::vector<JointPose>& outDelta);
void ApplyAdditive(std::vector<JointPose>& base, const std::vector<JointPose>& delta,
                   float weight, const std::vector<float>* jointMask = nullptr);

// Local -> model space, one forward pass (relies on parent-before-child order).
void ComputeGlobals(const Skeleton& skeleton, const std::vector<JointPose>& local,
                    std::vector<Math::Mat4>& outGlobal);

// palette[i] = InverseBind[i] * global[i] — the matrices the vertex shader
// (or CPU skinning path) applies to bind-pose vertices.
void ComputePalette(const Skeleton& skeleton, const std::vector<Math::Mat4>& global,
                    std::vector<Math::Mat4>& outPalette);

// Root motion extracted from a clip: the model-space horizontal travel of the
// clip's motion joint, relative to the clip start. The clip itself is edited
// to play IN PLACE; gameplay applies Delta() to the entity's world transform
// each frame — so characters really move, loop snap-back disappears, and
// travel speed exactly matches the animation (no foot sliding).
struct RootMotion {
    Vec4Channel Motion;              // absolute offset since clip start (linear keys)
    float       Duration    = 0.0f;
    uint32_t    SourceJoint = kInvalidJoint;

    bool HasMotion() const { return Motion.Times.size() > 1; }

    Math::Vec4 At(float t) const;                     // offset at time t (clamped)
    // Travel between two player times; t1 < t0 is treated as one loop wrap.
    Math::Vec4 Delta(float t0, float t1) const;
};

// Extracts root motion IN PLACE from `clip`: the motion joint is the first
// joint (parent-before-child order) with a keyed translation channel; the
// horizontal (model-space XZ) part of its travel is removed from the channel
// (values and, for cubic channels, tangents) and returned. Vertical motion
// stays in the clip. Clips with negligible horizontal travel (< 1mm) are left
// untouched and return an empty RootMotion (HasMotion() == false).
RootMotion ExtractRootMotion(const Skeleton& skeleton, AnimationClip& clip);

// Playback state for one clip instance. Sample() feeds the cursor cache back
// into SampleClip; SetTime/looping rewinds reset it (cursors only ever walk
// forward).
class AnimationPlayer {
public:
    void SetClip(const AnimationClip* clip, bool loop = true);
    const AnimationClip* Clip() const { return m_clip; }

    void  Update(float dt);        // advance; wraps (loop) or clamps at Duration
    void  SetTime(float t);        // scrub to an absolute time
    float Time() const { return m_time; }

    void SetLooping(bool loop) { m_loop = loop; }
    bool Looping() const       { return m_loop; }

    void Sample(const Skeleton& skeleton, std::vector<JointPose>& outLocal);

    // Events playback CROSSED during the last Update(), in firing order.
    // Semantics: half-open interval (prevTime, newTime]; a loop wrap fires the
    // tail of the timeline first, then the head (an event at exactly t=0 fires
    // on the wrap). SetTime is a seek and never fires; a dt spanning multiple
    // whole loops fires each event once. Cleared on every Update.
    const std::vector<const AnimationEvent*>& FiredEvents() const { return m_fired; }

private:
    const AnimationClip*  m_clip = nullptr;
    float                 m_time = 0.0f;
    bool                  m_loop = true;
    std::vector<uint32_t> m_cursors;
    std::vector<const AnimationEvent*> m_fired;
};

} // namespace SGE::Anim
