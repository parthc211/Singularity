// Numeric checks for SGE::Anim against hand-derived answers, plus the new
// Quat Slerp/Nlerp against DirectXMath.
//
// Test rig: 3-joint chain along +Y, each joint 1 unit above its parent.
//   joint 0 "root" at origin, joint 1 "mid" at (0,1,0), joint 2 "tip" at (0,2,0).
// Clip: mid rotates 0 -> 90 deg about Z between t=0 and t=1. Others unkeyed.
#include "Animation/Animation.h"

#include <DirectXMath.h>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace SGE::Anim;
using namespace SGE::Math;

static int g_failures = 0;

static void Check(const char* name, bool ok)
{
    std::printf("%-58s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

static bool Near(float a, float b, float eps = 1e-5f) { return std::fabs(a - b) < eps; }

static bool NearV(Vec4 v, float x, float y, float z, float eps = 1e-5f)
{
    return Near(v.x(), x, eps) && Near(v.y(), y, eps) && Near(v.z(), z, eps);
}

// Position of a joint-space origin in model space = row 3 of its global matrix.
static Vec4 Row3(const Mat4& m) { return Vec4(m.r[3]); }

static Skeleton MakeChain()
{
    Skeleton sk;
    sk.Names   = { "root", "mid", "tip" };
    sk.Parents = { kInvalidJoint, 0, 1 };
    sk.RestPose.resize(3);
    sk.RestPose[1].T = Vec4(0, 1, 0, 0);
    sk.RestPose[2].T = Vec4(0, 1, 0, 0);
    // Bind pose = rest pose. Bind globals are pure translations to y = 0, 1, 2,
    // so the inverse binds are the opposite translations.
    sk.InverseBind = { Mat4::Translation(0,  0, 0),
                       Mat4::Translation(0, -1, 0),
                       Mat4::Translation(0, -2, 0) };
    return sk;
}

static AnimationClip MakeSpinClip()
{
    AnimationClip clip;
    clip.Name     = "mid90z";
    clip.Duration = 1.0f;
    clip.Tracks.resize(3);
    clip.Tracks[1].Rotation.Times  = { 0.0f, 1.0f };
    clip.Tracks[1].Rotation.Values = {
        Quat(),                                                   // identity
        Quat::FromAxisAngle(Vec4(0, 0, 1, 0), DirectX::XM_PIDIV2) // 90 deg about Z
    };
    return clip;
}

int main()
{
    const Skeleton      sk   = MakeChain();
    const AnimationClip clip = MakeSpinClip();

    // 1. Slerp/Nlerp vs DirectXMath over random-ish fixed quats.
    {
        using namespace DirectX;
        Quat a = Normalize(Quat(0.3f, -0.5f, 0.7f, 0.2f));
        Quat b = Normalize(Quat(-0.6f, 0.1f, 0.4f, 0.8f));
        bool ok = true;
        for (float t : { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f }) {
            float fs[4], fd[4];
            _mm_storeu_ps(fs, Slerp(a, b, t).v);
            _mm_storeu_ps(fd, XMQuaternionSlerp(a.v, b.v, t));
            for (int i = 0; i < 4; ++i) ok = ok && Near(fs[i], fd[i], 1e-4f);
        }
        // Opposite hemisphere: must take the short way (match XM, which flips too).
        Quat bneg(-b.v.m128_f32[0], -b.v.m128_f32[1], -b.v.m128_f32[2], -b.v.m128_f32[3]);
        float fs[4], fd[4];
        _mm_storeu_ps(fs, Slerp(a, bneg, 0.5f).v);
        _mm_storeu_ps(fd, XMQuaternionSlerp(a.v, bneg.v, 0.5f));
        for (int i = 0; i < 4; ++i) ok = ok && Near(fs[i], fd[i], 1e-4f);
        Check("Slerp matches XMQuaternionSlerp (incl. hemisphere flip)", ok);
    }

    // 2. Channel sampling: t=0.5 must be a 45-degree Z rotation on the mid joint.
    {
        std::vector<JointPose> pose;
        SampleClip(sk, clip, 0.5f, pose);
        const Quat expect = Quat::FromAxisAngle(Vec4(0, 0, 1, 0), DirectX::XM_PIDIV4);
        float fs[4], fe[4];
        _mm_storeu_ps(fs, pose[1].R.v);
        _mm_storeu_ps(fe, expect.v);
        bool ok = true;
        for (int i = 0; i < 4; ++i) ok = ok && Near(fs[i], fe[i], 1e-5f);
        // Unkeyed joints hold the rest pose.
        ok = ok && NearV(pose[2].T, 0, 1, 0) && NearV(pose[0].T, 0, 0, 0)
                && NearV(pose[1].S, 1, 1, 1);
        Check("SampleClip: slerped key + rest-pose fallback", ok);
    }

    // 3. Clamp semantics outside the key range.
    {
        std::vector<JointPose> before, after;
        SampleClip(sk, clip, -0.5f, before);
        SampleClip(sk, clip,  9.0f, after);
        const Quat q90 = Quat::FromAxisAngle(Vec4(0, 0, 1, 0), DirectX::XM_PIDIV2);
        float fb[4], fa[4], f90[4];
        _mm_storeu_ps(fb, before[1].R.v);
        _mm_storeu_ps(fa, after[1].R.v);
        _mm_storeu_ps(f90, q90.v);
        bool ok = Near(fb[0], 0) && Near(fb[1], 0) && Near(fb[2], 0) && Near(fb[3], 1);
        for (int i = 0; i < 4; ++i) ok = ok && Near(fa[i], f90[i]);
        Check("SampleClip: clamps before first / after last key", ok);
    }

    // 4. Cursor cache: sweeping forward with a persistent cursor vector must
    //    produce identical results to stateless sampling at every step.
    {
        std::vector<uint32_t> cursors;
        bool ok = true;
        for (int step = 0; step <= 20; ++step) {
            const float t = 0.05f * float(step);
            std::vector<JointPose> cached, stateless;
            SampleClip(sk, clip, t, cached, &cursors);
            SampleClip(sk, clip, t, stateless);
            float fc[4], fsl[4];
            _mm_storeu_ps(fc, cached[1].R.v);
            _mm_storeu_ps(fsl, stateless[1].R.v);
            for (int i = 0; i < 4; ++i) ok = ok && Near(fc[i], fsl[i]);
        }
        Check("SampleClip: cursor cache == stateless sampling", ok);
    }

    // 5. Rest-pose globals: chain stacks to y = 0, 1, 2; palette == identity.
    {
        std::vector<JointPose> pose;
        SampleClip(sk, clip, 0.0f, pose);
        std::vector<Mat4> globals, palette;
        ComputeGlobals(sk, pose, globals);
        ComputePalette(sk, globals, palette);
        bool ok = NearV(Row3(globals[0]), 0, 0, 0)
               && NearV(Row3(globals[1]), 0, 1, 0)
               && NearV(Row3(globals[2]), 0, 2, 0);
        for (int j = 0; j < 3 && ok; ++j) {
            const Mat4 I = Mat4::Identity();
            float fp[4], fi[4];
            for (int r = 0; r < 4; ++r) {
                _mm_storeu_ps(fp, palette[j].r[r]);
                _mm_storeu_ps(fi, I.r[r]);
                for (int c = 0; c < 4; ++c) ok = ok && Near(fp[c], fi[c]);
            }
        }
        Check("Bind pose: globals stack, palette is identity", ok);
    }

    // 6. Posed globals at t=1: mid bends 90 deg, so the tip swings from (0,2,0)
    //    to (-1,1,0). A bind-pose vertex at the tip must follow via the palette.
    {
        std::vector<JointPose> pose;
        SampleClip(sk, clip, 1.0f, pose);
        std::vector<Mat4> globals, palette;
        ComputeGlobals(sk, pose, globals);
        ComputePalette(sk, globals, palette);

        bool ok = NearV(Row3(globals[2]), -1, 1, 0);
        const Vec4 skinned = Transform(Vec4(0, 2, 0, 1), palette[2]);
        ok = ok && NearV(skinned, -1, 1, 0);
        // A vertex 100% weighted to the STATIC root must not move.
        const Vec4 still = Transform(Vec4(0.3f, 0.1f, 0.2f, 1), palette[0]);
        ok = ok && NearV(still, 0.3f, 0.1f, 0.2f);
        Check("t=1: tip swings to (-1,1,0); root-skinned vertex fixed", ok);
    }

    // 7. BlendPoses at 0.5 between rest and t=1 pose == the t=0.5 sample
    //    (one linear rotation channel: slerp midpoint == nlerp midpoint).
    {
        std::vector<JointPose> restPose, endPose, blended, sampled;
        SampleClip(sk, clip, 0.0f, restPose);
        SampleClip(sk, clip, 1.0f, endPose);
        BlendPoses(restPose, endPose, 0.5f, blended);
        SampleClip(sk, clip, 0.5f, sampled);

        std::vector<Mat4> gb, gs;
        ComputeGlobals(sk, blended, gb);
        ComputeGlobals(sk, sampled, gs);
        const Vec4 pb = Row3(gb[2]), ps = Row3(gs[2]);
        bool ok = NearV(pb, ps.x(), ps.y(), ps.z(), 1e-4f);
        // And the hand-derived position: rotate (0,1,0) by 45 deg about Z + (0,1,0).
        const float s = std::sqrt(0.5f);
        ok = ok && NearV(pb, -s, 1.0f + s, 0.0f, 1e-4f);
        Check("BlendPoses(0.5) == sampled midpoint == hand-derived", ok);
    }

    // 8. AnimationPlayer: looping wrap and backwards scrub stay consistent.
    {
        AnimationPlayer player;
        player.SetClip(&clip, true);
        std::vector<JointPose> a, b;

        player.Update(0.7f);                    // t = 0.7
        player.Sample(sk, a);
        player.Update(0.8f);                    // t = 1.5 -> wraps to 0.5
        player.Sample(sk, b);
        bool ok = Near(player.Time(), 0.5f);
        std::vector<JointPose> ref;
        SampleClip(sk, clip, 0.5f, ref);
        float fb[4], fr[4];
        _mm_storeu_ps(fb, b[1].R.v);
        _mm_storeu_ps(fr, ref[1].R.v);
        for (int i = 0; i < 4; ++i) ok = ok && Near(fb[i], fr[i]);

        player.SetTime(0.1f);                   // backwards scrub resets cursors
        player.Sample(sk, a);
        SampleClip(sk, clip, 0.1f, ref);
        _mm_storeu_ps(fb, a[1].R.v);
        _mm_storeu_ps(fr, ref[1].R.v);
        for (int i = 0; i < 4; ++i) ok = ok && Near(fb[i], fr[i]);
        Check("AnimationPlayer: loop wrap + backwards scrub", ok);
    }

    // 9. Additive layering: the delta of (pose vs ref) applied onto ref at
    //    weight 1 must reconstruct the pose EXACTLY; weight 0 is a no-op; a
    //    zeroed mask entry pins that joint to the base.
    {
        std::vector<JointPose> ref, posed, delta, out;
        SampleClip(sk, clip, 0.0f, ref);      // identity rotations
        SampleClip(sk, clip, 1.0f, posed);    // mid joint bent 90 degrees

        MakeAdditive(posed, ref, delta);

        auto sameQuat = [](Quat a, Quat b) {
            return std::fabs(Dot(a, b)) > 1.0f - 1e-6f;   // q == -q
        };

        out = ref;
        ApplyAdditive(out, delta, 1.0f);
        bool ok = true;
        for (size_t j = 0; j < out.size(); ++j)
            ok = ok && sameQuat(out[j].R, posed[j].R)
                    && NearV(out[j].T, posed[j].T.x(), posed[j].T.y(), posed[j].T.z());
        Check("Additive: delta applied at w=1 reconstructs the pose", ok);

        out = ref;
        ApplyAdditive(out, delta, 0.0f);
        ok = true;
        for (size_t j = 0; j < out.size(); ++j)
            ok = ok && sameQuat(out[j].R, ref[j].R);
        Check("Additive: w=0 is a no-op", ok);

        // Half weight on the mid joint = 45-degree bend (slerp midpoint).
        out = ref;
        ApplyAdditive(out, delta, 0.5f);
        const Quat q45 = Quat::FromAxisAngle(Vec4(0, 0, 1, 0), DirectX::XM_PIDIV4);
        Check("Additive: w=0.5 gives the slerp midpoint", sameQuat(out[1].R, q45));

        // Mask: zero out the mid joint — it must stay at the base pose.
        std::vector<float> mask = { 1.0f, 0.0f, 1.0f };
        out = ref;
        ApplyAdditive(out, delta, 1.0f, &mask);
        Check("Additive: masked joint pinned to base", sameQuat(out[1].R, ref[1].R));
    }

    // 10. Animation events: crossing fires exactly once and in order; wraps
    //     fire tail-then-head; seeks never fire; non-loop end fires once.
    {
        AnimationClip evClip = MakeSpinClip();   // duration 1.0
        evClip.Events = { { 0.0f, "loopstart" }, { 0.25f, "quarter" },
                          { 0.5f, "half" }, { 0.95f, "tail" } };

        AnimationPlayer p;
        p.SetClip(&evClip, true);

        p.Update(0.30f);   // 0 -> 0.30: crosses "quarter" only ("loopstart" at
                           // t=0 is not in the half-open interval (0, 0.3])
        bool ok = p.FiredEvents().size() == 1 && p.FiredEvents()[0]->Name == "quarter";
        Check("Events: single crossing fires exactly once", ok);

        p.Update(0.10f);   // 0.30 -> 0.40: nothing
        Check("Events: no crossing, no fire", p.FiredEvents().empty());

        p.Update(0.58f);   // 0.40 -> 0.98: "half" then "tail", in order
        ok = p.FiredEvents().size() == 2 &&
             p.FiredEvents()[0]->Name == "half" && p.FiredEvents()[1]->Name == "tail";
        Check("Events: multiple crossings fire in order", ok);

        p.Update(0.10f);   // 0.98 -> wraps to 0.08: fires "loopstart" (t=0 on
                           // the wrap) — tail already fired, head has nothing else
        ok = p.FiredEvents().size() == 1 && p.FiredEvents()[0]->Name == "loopstart";
        Check("Events: loop wrap fires the head (incl. t=0)", ok);

        p.SetTime(0.9f);   // seek: silent
        Check("Events: SetTime never fires", p.FiredEvents().empty());

        // Non-looping: crossing the end fires what's left exactly once, then
        // the clamped player stays silent.
        AnimationPlayer q;
        q.SetClip(&evClip, false);
        q.SetTime(0.6f);
        q.Update(1.0f);    // 0.6 -> clamped 1.0: crosses "tail"
        ok = q.FiredEvents().size() == 1 && q.FiredEvents()[0]->Name == "tail";
        q.Update(0.5f);    // still clamped: empty interval
        ok = ok && q.FiredEvents().empty();
        Check("Events: non-loop end fires once, then stays silent", ok);
    }

    // 11. Root-motion extraction: horizontal travel moves out of the clip,
    //     vertical stays; At/Delta are exact incl. loop wrap; in-place clips
    //     are untouched.
    {
        AnimationClip travel = MakeSpinClip();   // duration 1.0
        travel.Tracks[0].Translation.Times  = { 0.0f, 1.0f };
        travel.Tracks[0].Translation.Values = { Vec4(0, 0, 0, 0), Vec4(2, 0.5f, 0, 0) };

        RootMotion rm = ExtractRootMotion(sk, travel);
        Check("RootMotion: extraction finds the keyed root joint",
              rm.HasMotion() && rm.SourceJoint == 0);

        // Clip is now in place horizontally, vertical motion preserved.
        const Vec4 endKey = travel.Tracks[0].Translation.Values.back();
        Check("RootMotion: clip keeps vertical, loses horizontal",
              NearV(endKey, 0, 0.5f, 0));

        // Absolute offsets and step deltas, including the wrap case:
        // wrap 0.75 -> 0.25 = tail (0.5) + head (0.5) = 1.0 total.
        bool ok = NearV(rm.At(1.0f), 2, 0, 0) && NearV(rm.At(0.5f), 1, 0, 0)
               && NearV(rm.Delta(0.25f, 0.75f), 1, 0, 0)
               && NearV(rm.Delta(0.75f, 0.25f), 1, 0, 0);
        Check("RootMotion: At/Delta exact, wrap-aware", ok);

        // The stripped clip really animates in place: root global at t=1
        // sits at (0, 0.5, 0), not (2, 0.5, 0).
        std::vector<JointPose> pose;
        SampleClip(sk, travel, 1.0f, pose);
        std::vector<Mat4> globals;
        ComputeGlobals(sk, pose, globals);
        float g[4]; _mm_storeu_ps(g, globals[0].r[3]);
        Check("RootMotion: stripped clip plays in place",
              Near(g[0], 0) && Near(g[1], 0.5f) && Near(g[2], 0));

        // A clip with no horizontal travel is left byte-identical.
        AnimationClip inPlace = MakeSpinClip();
        inPlace.Tracks[0].Translation.Times  = { 0.0f, 1.0f };
        inPlace.Tracks[0].Translation.Values = { Vec4(0, 0, 0, 0), Vec4(0, 0.7f, 0, 0) };
        RootMotion none = ExtractRootMotion(sk, inPlace);
        Check("RootMotion: in-place clip untouched, HasMotion false",
              !none.HasMotion() &&
              NearV(inPlace.Tracks[0].Translation.Values.back(), 0, 0.7f, 0));
    }

    std::printf("\n%s\n", g_failures == 0 ? "ALL PASS" : "FAILURES");
    return g_failures;
}
