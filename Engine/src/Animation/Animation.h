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

// Local -> model space, one forward pass (relies on parent-before-child order).
void ComputeGlobals(const Skeleton& skeleton, const std::vector<JointPose>& local,
                    std::vector<Math::Mat4>& outGlobal);

// palette[i] = InverseBind[i] * global[i] — the matrices the vertex shader
// (or CPU skinning path) applies to bind-pose vertices.
void ComputePalette(const Skeleton& skeleton, const std::vector<Math::Mat4>& global,
                    std::vector<Math::Mat4>& outPalette);

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

private:
    const AnimationClip*  m_clip = nullptr;
    float                 m_time = 0.0f;
    bool                  m_loop = true;
    std::vector<uint32_t> m_cursors;
};

} // namespace SGE::Anim
