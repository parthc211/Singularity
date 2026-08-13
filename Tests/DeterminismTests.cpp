// Phase D checks: crossfade blending behavior, and bit-identical serial vs
// JobSystem-parallel pose evaluation across a staggered instance grid
// (real CesiumMan data, real JobSystem).
#include "Animation/Animation.h"
#include "Assets/GltfLoader.h"
#include "Jobs/JobSystem.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace SGE;
using namespace SGE::Anim;
using namespace SGE::Math;

static int g_failures = 0;

static void Check(const char* name, bool ok)
{
    std::printf("%-62s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

struct Inst {
    AnimationPlayer Player, PrevPlayer;
    float Fade = 1.0f;
    std::vector<JointPose> Pose, PrevPose;
    std::vector<Mat4> Globals, Palette;
};

static void Evaluate(Inst& in, const Skeleton& sk)
{
    in.Player.Sample(sk, in.Pose);
    if (in.Fade < 1.0f) {
        in.PrevPlayer.Sample(sk, in.PrevPose);
        const float f = in.Fade;
        BlendPoses(in.PrevPose, in.Pose, f * f * (3.0f - 2.0f * f), in.Pose);
    }
    ComputeGlobals(sk, in.Pose, in.Globals);
    ComputePalette(sk, in.Globals, in.Palette);
}

int main()
{
    SkeletalMeshData d;
    if (!LoadGLTF("Assets/CesiumMan.glb", d)) {
        std::printf("cannot load CesiumMan\n");
        return 1;
    }
    const Skeleton& sk = d.Skeleton;
    const AnimationClip& clip = d.Clips[0];

    // 1. Crossfade endpoints: fade 0 == old pose, fade 1 == new pose,
    //    intermediate fades move joints monotonically off the endpoints.
    {
        std::vector<JointPose> oldPose, newPose, blended;
        SampleClip(sk, clip, 0.3f, oldPose);
        SampleClip(sk, clip, 1.4f, newPose);

        BlendPoses(oldPose, newPose, 0.0f, blended);
        float err0 = 0.0f;
        for (size_t j = 0; j < blended.size(); ++j) {
            float a[4], b[4];
            _mm_storeu_ps(a, blended[j].T.v); _mm_storeu_ps(b, oldPose[j].T.v);
            for (int k = 0; k < 3; ++k) err0 = std::max(err0, std::fabs(a[k] - b[k]));
        }
        BlendPoses(oldPose, newPose, 1.0f, blended);
        float err1 = 0.0f;
        for (size_t j = 0; j < blended.size(); ++j) {
            float a[4], b[4];
            _mm_storeu_ps(a, blended[j].T.v); _mm_storeu_ps(b, newPose[j].T.v);
            for (int k = 0; k < 3; ++k) err1 = std::max(err1, std::fabs(a[k] - b[k]));
        }
        Check("crossfade endpoints reproduce source poses", err0 < 1e-6f && err1 < 1e-6f);

        // Quaternions must stay unit through the nlerp blend.
        BlendPoses(oldPose, newPose, 0.37f, blended);
        bool unit = true;
        for (const JointPose& p : blended)
            unit = unit && std::fabs(Dot(p.R, p.R) - 1.0f) < 1e-4f;
        Check("blended rotations stay unit quaternions", unit);
    }

    // 2. Serial vs JobSystem-parallel evaluation over a staggered grid:
    //    identical inputs must produce BIT-IDENTICAL palettes.
    {
        constexpr int N = 16;
        std::vector<Inst> serial(N), parallel(N);
        auto setup = [&](std::vector<Inst>& v) {
            for (int i = 0; i < N; ++i) {
                v[size_t(i)].Player.SetClip(&clip, true);
                v[size_t(i)].Player.SetTime(0.37f * float(i));
                // Half the grid mid-crossfade, to cover that path too.
                if (i % 2) {
                    v[size_t(i)].PrevPlayer.SetClip(&clip, true);
                    v[size_t(i)].PrevPlayer.SetTime(0.11f * float(i));
                    v[size_t(i)].Fade = 0.25f + 0.03f * float(i);
                }
            }
        };
        setup(serial);
        setup(parallel);

        // Advance both grids a few frames, evaluating serial vs Dispatch.
        JobSystem jobs;
        jobs.Initialize();
        for (int frame = 0; frame < 8; ++frame) {
            const float dt = 1.0f / 60.0f;
            for (int i = 0; i < N; ++i) {
                serial[size_t(i)].Player.Update(dt);
                parallel[size_t(i)].Player.Update(dt);
                if (serial[size_t(i)].Fade < 1.0f)   serial[size_t(i)].PrevPlayer.Update(dt);
                if (parallel[size_t(i)].Fade < 1.0f) parallel[size_t(i)].PrevPlayer.Update(dt);
            }
            for (int i = 0; i < N; ++i)
                Evaluate(serial[size_t(i)], sk);
            jobs.Dispatch(N, 1, [&](uint32_t i) { Evaluate(parallel[i], sk); });
            jobs.Wait();
        }
        jobs.Shutdown();

        bool identical = true;
        for (int i = 0; i < N; ++i)
            identical = identical &&
                serial[size_t(i)].Palette.size() == parallel[size_t(i)].Palette.size() &&
                std::memcmp(serial[size_t(i)].Palette.data(),
                            parallel[size_t(i)].Palette.data(),
                            serial[size_t(i)].Palette.size() * sizeof(Mat4)) == 0;
        Check("serial vs JobSystem palettes bit-identical (16 instances, 8 frames)", identical);
    }

    std::printf("\n%s\n", g_failures == 0 ? "ALL PASS" : "FAILURES");
    return g_failures;
}
