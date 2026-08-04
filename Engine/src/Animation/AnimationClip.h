#pragma once
// ---------------------------------------------------------------------------
// AnimationClip — keyframe data, one track per skeleton joint.
//
// Each track has independent T/R/S channels: a channel is (times, values)
// arrays of equal length with strictly increasing times. Empty channels mean
// "this joint holds its skeleton rest value" — typical clips key only a few
// joints' scales, for example, and this keeps them sparse.
//
// Sampling semantics (see Animation.h): clamp before the first / after the
// last key, piecewise interpolation between keys (lerp for T/S, slerp for R).
// ---------------------------------------------------------------------------
#include "Math/SimdMath.h"

#include <string>
#include <vector>

namespace SGE::Anim {

struct Vec4Channel {
    std::vector<float>      Times;
    std::vector<Math::Vec4> Values;
};

struct QuatChannel {
    std::vector<float>      Times;
    std::vector<Math::Quat> Values;
};

struct JointTrack {
    Vec4Channel Translation;
    QuatChannel Rotation;
    Vec4Channel Scale;
};

struct AnimationClip {
    std::string             Name;
    float                   Duration = 0.0f;  // seconds; max key time across channels
    std::vector<JointTrack> Tracks;           // indexed like Skeleton joints
};

} // namespace SGE::Anim
