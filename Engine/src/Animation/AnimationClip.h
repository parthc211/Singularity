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
// last key; between keys the channel's Interp mode applies — Linear (lerp for
// T/S, slerp for R), Step (hold the previous key), or CubicSpline (glTF-style
// Hermite: per-key in/out tangents scaled by the key interval; rotations
// interpolate componentwise and renormalize, per the glTF spec).
// ---------------------------------------------------------------------------
#include "Math/SimdMath.h"

#include <cstdint>
#include <string>
#include <vector>

namespace SGE::Anim {

enum class Interpolation : uint8_t { Linear, Step, CubicSpline };

struct Vec4Channel {
    Interpolation           Interp = Interpolation::Linear;
    std::vector<float>      Times;
    std::vector<Math::Vec4> Values;
    std::vector<Math::Vec4> InTan;    // CubicSpline only: per-key in-tangents
    std::vector<Math::Vec4> OutTan;   //                   and out-tangents
};

struct QuatChannel {
    Interpolation           Interp = Interpolation::Linear;
    std::vector<float>      Times;
    std::vector<Math::Quat> Values;
    std::vector<Math::Quat> InTan;    // raw (unnormalized) component tangents
    std::vector<Math::Quat> OutTan;
};

struct JointTrack {
    Vec4Channel Translation;
    QuatChannel Rotation;
    Vec4Channel Scale;
};

// A named marker on the clip timeline ("footstep", "throw", ...). Events are
// data only; AnimationPlayer::Update reports the ones playback crossed and
// the application decides what they mean.
struct AnimationEvent {
    float       Time = 0.0f;   // seconds, within [0, Duration]
    std::string Name;
};

struct AnimationClip {
    std::string                 Name;
    float                       Duration = 0.0f;  // seconds; max key time across channels
    std::vector<JointTrack>     Tracks;           // indexed like Skeleton joints
    std::vector<AnimationEvent> Events;           // sorted by Time (keep sorted when editing)
};

} // namespace SGE::Anim
