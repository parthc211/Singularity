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

Math::Vec4 SampleVec4(const Vec4Channel& c, Math::Vec4 rest, float t, uint32_t& cursor)
{
    if (c.Times.empty()) return rest;
    const uint32_t i = FindKey(c.Times, t, cursor);
    if (i == kNoKey) { cursor = 0; return c.Values.front(); }
    cursor = i;
    if (i + 1 >= c.Times.size()) return c.Values.back();
    const float f = (t - c.Times[i]) / (c.Times[i + 1] - c.Times[i]);
    return Math::Lerp(c.Values[i], c.Values[i + 1], f);
}

Math::Quat SampleQuat(const QuatChannel& c, Math::Quat rest, float t, uint32_t& cursor)
{
    if (c.Times.empty()) return rest;
    const uint32_t i = FindKey(c.Times, t, cursor);
    if (i == kNoKey) { cursor = 0; return c.Values.front(); }
    cursor = i;
    if (i + 1 >= c.Times.size()) return c.Values.back();
    const float f = (t - c.Times[i]) / (c.Times[i + 1] - c.Times[i]);
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

// --- AnimationPlayer -----------------------------------------------------------

void AnimationPlayer::SetClip(const AnimationClip* clip, bool loop)
{
    m_clip = clip;
    m_loop = loop;
    m_time = 0.0f;
    m_cursors.assign(m_cursors.size(), kNoKey);
}

void AnimationPlayer::Update(float dt)
{
    if (!m_clip || m_clip->Duration <= 0.0f) return;
    const float prev = m_time;
    m_time += dt;
    if (m_loop) {
        if (m_time >= m_clip->Duration) {
            m_time = std::fmod(m_time, m_clip->Duration);
            m_cursors.assign(m_cursors.size(), kNoKey);   // wrapped: time went backwards
        }
    } else {
        m_time = std::clamp(m_time, 0.0f, m_clip->Duration);
    }
    if (m_time < prev && !m_loop)
        m_cursors.assign(m_cursors.size(), kNoKey);
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
