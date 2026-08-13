#include "Animation/Animation.h"

#include <algorithm>
#include <cmath>

namespace SGE::Anim {

namespace {

constexpr uint32_t kNoKey = 0xFFFFFFFFu;

// Index of the last key with Times[i] <= t, or kNoKey if t precedes the first.
// The hint (last frame's result) makes steady playback O(keys advanced); a
// stale/backwards hint falls back to one binary search.
uint32_t FindKey(const std::vector<float>& times, float t, uint32_t hint)
{
    const uint32_t n = static_cast<uint32_t>(times.size());
    if (n == 0 || t < times[0]) return kNoKey;

    uint32_t i;
    if (hint < n && times[hint] <= t) {
        i = hint;
        while (i + 1 < n && times[i + 1] <= t) ++i;
    } else {
        i = static_cast<uint32_t>(
                std::upper_bound(times.begin(), times.end(), t) - times.begin()) - 1;
    }
    return i;
}

// Hermite basis for glTF CUBICSPLINE (tangents are per-second, so the tangent
// terms scale by the key interval td).
struct Hermite { float h00, h10, h01, h11; };
Hermite HermiteBasis(float s)
{
    const float s2 = s * s, s3 = s2 * s;
    return { 2.0f * s3 - 3.0f * s2 + 1.0f,
             s3 - 2.0f * s2 + s,
             -2.0f * s3 + 3.0f * s2,
             s3 - s2 };
}

Math::Vec4 SampleVec4(const Vec4Channel& c, Math::Vec4 rest, float t, uint32_t& cursor)
{
    if (c.Times.empty()) return rest;
    const uint32_t i = FindKey(c.Times, t, cursor);
    if (i == kNoKey) { cursor = 0; return c.Values.front(); }
    cursor = i;
    if (i + 1 >= c.Times.size()) return c.Values.back();
    const float td = c.Times[i + 1] - c.Times[i];
    const float f  = (t - c.Times[i]) / td;

    if (c.Interp == Interpolation::Step)
        return c.Values[i];
    if (c.Interp == Interpolation::CubicSpline &&
        c.OutTan.size() == c.Values.size() && c.InTan.size() == c.Values.size()) {
        const Hermite h = HermiteBasis(f);
        return c.Values[i]     * h.h00 + c.OutTan[i]    * (h.h10 * td)
             + c.Values[i + 1] * h.h01 + c.InTan[i + 1] * (h.h11 * td);
    }
    return Math::Lerp(c.Values[i], c.Values[i + 1], f);
}

Math::Quat SampleQuat(const QuatChannel& c, Math::Quat rest, float t, uint32_t& cursor)
{
    if (c.Times.empty()) return rest;
    const uint32_t i = FindKey(c.Times, t, cursor);
    if (i == kNoKey) { cursor = 0; return c.Values.front(); }
    cursor = i;
    if (i + 1 >= c.Times.size()) return c.Values.back();
    const float td = c.Times[i + 1] - c.Times[i];
    const float f  = (t - c.Times[i]) / td;

    if (c.Interp == Interpolation::Step)
        return c.Values[i];
    if (c.Interp == Interpolation::CubicSpline &&
        c.OutTan.size() == c.Values.size() && c.InTan.size() == c.Values.size()) {
        // Per the glTF spec: componentwise Hermite, then renormalize.
        const Hermite h = HermiteBasis(f);
        const Math::Quat q = c.Values[i]     * h.h00 + c.OutTan[i]    * (h.h10 * td)
                           + c.Values[i + 1] * h.h01 + c.InTan[i + 1] * (h.h11 * td);
        return Math::Normalize(q);
    }
    return Math::Slerp(c.Values[i], c.Values[i + 1], f);
}

// Compose S*R*T directly (row-vector convention): rotation rows scaled by the
// scale components, translation in row 3. Saves two full 4x4 multiplies per
// joint over Mul(Mul(S,R),T).
Math::Mat4 LocalMatrix(const JointPose& p)
{
    Math::Mat4 m = Math::ToMatrix(p.R);
    m.r[0] = _mm_mul_ps(m.r[0], _mm_set1_ps(p.S.x()));
    m.r[1] = _mm_mul_ps(m.r[1], _mm_set1_ps(p.S.y()));
    m.r[2] = _mm_mul_ps(m.r[2], _mm_set1_ps(p.S.z()));
    m.r[3] = _mm_set_ps(1.0f, p.T.z(), p.T.y(), p.T.x());
    return m;
}

} // namespace

void SampleClip(const Skeleton& skeleton, const AnimationClip& clip, float t,
                std::vector<JointPose>& outLocal, std::vector<uint32_t>* cursors)
{
    const uint32_t joints = skeleton.JointCount();
    outLocal.resize(joints);

    // Stateless fallback: local scratch, every lookup binary-searches.
    std::vector<uint32_t> scratch;
    std::vector<uint32_t>& cur = cursors ? *cursors : scratch;
    cur.resize(size_t(joints) * 3, kNoKey);

    for (uint32_t j = 0; j < joints; ++j) {
        const JointPose& rest = skeleton.RestPose[j];
        if (j >= clip.Tracks.size()) { outLocal[j] = rest; continue; }

        const JointTrack& track = clip.Tracks[j];
        outLocal[j].T = SampleVec4(track.Translation, rest.T, t, cur[size_t(j) * 3 + 0]);
        outLocal[j].R = SampleQuat(track.Rotation,    rest.R, t, cur[size_t(j) * 3 + 1]);
        outLocal[j].S = SampleVec4(track.Scale,       rest.S, t, cur[size_t(j) * 3 + 2]);
    }
}

void BlendPoses(const std::vector<JointPose>& a, const std::vector<JointPose>& b,
                float t, std::vector<JointPose>& out)
{
    out.resize(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        out[i].T = Math::Lerp(a[i].T, b[i].T, t);
        out[i].R = Math::Nlerp(a[i].R, b[i].R, t);
        out[i].S = Math::Lerp(a[i].S, b[i].S, t);
    }
}

void MakeAdditive(const std::vector<JointPose>& pose,
                  const std::vector<JointPose>& reference,
                  std::vector<JointPose>& outDelta)
{
    const size_t n = std::min(pose.size(), reference.size());
    outDelta.resize(pose.size());
    for (size_t i = 0; i < n; ++i) {
        const JointPose& p = pose[i];
        const JointPose& r = reference[i];
        JointPose d;
        d.T = p.T + r.T * -1.0f;
        d.R = Math::Mul(Math::Conjugate(r.R), p.R);   // ref^-1 * pose (unit quats)
        d.S = Math::Vec4(p.S.x() / std::max(r.S.x(), 1e-6f),
                         p.S.y() / std::max(r.S.y(), 1e-6f),
                         p.S.z() / std::max(r.S.z(), 1e-6f), 0.0f);
        outDelta[i] = d;
    }
}

void ApplyAdditive(std::vector<JointPose>& base, const std::vector<JointPose>& delta,
                   float weight, const std::vector<float>* jointMask)
{
    const size_t n = std::min(base.size(), delta.size());
    for (size_t i = 0; i < n; ++i) {
        const float w = weight * (jointMask && i < jointMask->size() ? (*jointMask)[i] : 1.0f);
        if (w <= 0.0f) continue;
        const JointPose& d = delta[i];
        JointPose& b = base[i];

        b.T += d.T * w;
        // Partial delta rotation: slerp identity -> delta, composed onto base.
        // At w == 1 this reconstructs the layered pose exactly (see invariant).
        b.R = Math::Normalize(Math::Mul(b.R, Math::Slerp(Math::Quat(), d.R, w)));
        b.S = Math::Vec4(b.S.x() * (1.0f + (d.S.x() - 1.0f) * w),
                         b.S.y() * (1.0f + (d.S.y() - 1.0f) * w),
                         b.S.z() * (1.0f + (d.S.z() - 1.0f) * w), 0.0f);
    }
}

void ComputeGlobals(const Skeleton& skeleton, const std::vector<JointPose>& local,
                    std::vector<Math::Mat4>& outGlobal)
{
    const uint32_t joints = skeleton.JointCount();
    outGlobal.resize(joints);
    for (uint32_t i = 0; i < joints; ++i) {
        const Math::Mat4 m = LocalMatrix(local[i]);
        const uint32_t parent = skeleton.Parents[i];
        outGlobal[i] = (parent == kInvalidJoint) ? m : Math::Mul(m, outGlobal[parent]);
    }
}

void ComputePalette(const Skeleton& skeleton, const std::vector<Math::Mat4>& global,
                    std::vector<Math::Mat4>& outPalette)
{
    const uint32_t joints = skeleton.JointCount();
    outPalette.resize(joints);
    for (uint32_t i = 0; i < joints; ++i)
        outPalette[i] = Math::Mul(skeleton.InverseBind[i], global[i]);
}

// --- root motion ---------------------------------------------------------------

Math::Vec4 RootMotion::At(float t) const
{
    uint32_t cursor = kNoKey;   // stateless: one binary search
    return SampleVec4(Motion, Math::Vec4(), t, cursor);
}

Math::Vec4 RootMotion::Delta(float t0, float t1) const
{
    if (!HasMotion()) return Math::Vec4();
    if (t1 >= t0)
        return At(t1) + At(t0) * -1.0f;
    // Wrapped: tail of the loop plus the head. (A dt spanning several whole
    // loops still counts one — same collapse as the player's fmod.)
    return (At(Duration) + At(t0) * -1.0f) + At(t1);
}

RootMotion ExtractRootMotion(const Skeleton& skeleton, AnimationClip& clip)
{
    RootMotion rm;
    rm.Duration = clip.Duration;

    // Motion joint: first joint with a keyed translation channel (for rigs
    // this is the hips/pelvis; the pseudo scene-prefix root is never keyed).
    uint32_t joint = kInvalidJoint;
    for (uint32_t j = 0; j < clip.Tracks.size() && j < skeleton.JointCount(); ++j) {
        if (clip.Tracks[j].Translation.Times.size() > 1) { joint = j; break; }
    }
    if (joint == kInvalidJoint) return rm;
    Vec4Channel& ch = clip.Tracks[joint].Translation;

    // The joint's translation is in its parent's space; rotate deltas into
    // model space through the parent chain's REST globals (static above the
    // motion joint by construction — it is the first keyed joint).
    std::vector<Math::Mat4> restGlobals;
    ComputeGlobals(skeleton, skeleton.RestPose, restGlobals);
    const uint32_t parent   = skeleton.Parents[joint];
    Math::Mat4 toModel      = parent == kInvalidJoint ? Math::Mat4::Identity()
                                                      : restGlobals[parent];
    toModel.r[3]            = _mm_set_ps(1, 0, 0, 0);   // rotation/scale only
    const Math::Mat4 toLocal = Math::Inverse3x3(toModel);

    // Horizontal model-space travel per key, relative to key 0.
    const Math::Vec4 origin = ch.Values.front();
    std::vector<Math::Vec4> horizontal(ch.Values.size());
    float maxTravel = 0.0f;
    for (size_t k = 0; k < ch.Values.size(); ++k) {
        const Math::Vec4 model = Math::Transform(ch.Values[k] + origin * -1.0f, toModel);
        horizontal[k] = Math::Vec4(model.x(), 0.0f, model.z(), 0.0f);
        maxTravel = std::max(maxTravel, Math::Length3(horizontal[k]));
    }
    if (maxTravel < 1e-3f) return rm;   // authored in place — leave untouched

    // Strip the horizontal travel from the clip (vertical motion stays)...
    for (size_t k = 0; k < ch.Values.size(); ++k)
        ch.Values[k] = ch.Values[k] + Math::Transform(horizontal[k], toLocal) * -1.0f;
    // ...including cubic tangents (slopes must lose the same components).
    if (ch.Interp == Interpolation::CubicSpline &&
        ch.InTan.size() == ch.Values.size() && ch.OutTan.size() == ch.Values.size()) {
        auto stripTangent = [&](Math::Vec4& tan) {
            const Math::Vec4 m = Math::Transform(tan, toModel);
            tan = tan + Math::Transform(Math::Vec4(m.x(), 0.0f, m.z(), 0.0f), toLocal) * -1.0f;
        };
        for (size_t k = 0; k < ch.Values.size(); ++k) {
            stripTangent(ch.InTan[k]);
            stripTangent(ch.OutTan[k]);
        }
    }

    rm.SourceJoint   = joint;
    rm.Motion.Times  = ch.Times;
    rm.Motion.Values = std::move(horizontal);
    return rm;
}

// --- AnimationPlayer -----------------------------------------------------------

void AnimationPlayer::SetClip(const AnimationClip* clip, bool loop)
{
    m_clip = clip;
    m_loop = loop;
    m_time = 0.0f;
    m_cursors.assign(m_cursors.size(), kNoKey);
    m_fired.clear();
}

void AnimationPlayer::Update(float dt)
{
    m_fired.clear();
    if (!m_clip || m_clip->Duration <= 0.0f) return;
    const float prev    = m_time;
    bool        wrapped = false;
    m_time += dt;
    if (m_loop) {
        if (m_time >= m_clip->Duration) {
            m_time = std::fmod(m_time, m_clip->Duration);
            m_cursors.assign(m_cursors.size(), kNoKey);   // wrapped: time went backwards
            wrapped = true;
        }
    } else {
        m_time = std::clamp(m_time, 0.0f, m_clip->Duration);
    }
    if (m_time < prev && !m_loop)
        m_cursors.assign(m_cursors.size(), kNoKey);

    // Fire events crossed by this step (see FiredEvents() for the semantics).
    if (dt > 0.0f && !m_clip->Events.empty()) {
        auto collect = [&](float lo, float hi) {
            for (const AnimationEvent& e : m_clip->Events)
                if (e.Time > lo && e.Time <= hi)
                    m_fired.push_back(&e);
        };
        if (wrapped) {
            collect(prev, m_clip->Duration);   // tail of the timeline
            collect(-1.0f, m_time);            // head (includes t == 0 exactly)
        } else {
            collect(prev, m_time);
        }
    }
}

void AnimationPlayer::SetTime(float t)
{
    if (m_clip && m_clip->Duration > 0.0f) {
        t = m_loop ? std::fmod(std::max(t, 0.0f), m_clip->Duration)
                   : std::clamp(t, 0.0f, m_clip->Duration);
    }
    if (t < m_time)
        m_cursors.assign(m_cursors.size(), kNoKey);       // scrubbed backwards
    m_time = t;
    m_fired.clear();                                      // a seek never fires events
}

void AnimationPlayer::Sample(const Skeleton& skeleton, std::vector<JointPose>& outLocal)
{
    if (!m_clip) {
        outLocal = skeleton.RestPose;
        return;
    }
    SampleClip(skeleton, *m_clip, m_time, outLocal, &m_cursors);
}

} // namespace SGE::Anim
