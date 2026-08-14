// Headless checks for sleeping/islands — and the physics engine's standing
// determinism guarantee, now including sleep/wake transitions.
#include "Physics/PhysicsWorld.h"
#include "Jobs/JobSystem.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace SGE;
using namespace SGE::Physics;
using namespace SGE::Math;

static int g_failures = 0;

static void Check(const char* name, bool ok)
{
    std::printf("%-62s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

static void StepSeconds(PhysicsWorld& w, float seconds, JobSystem* jobs = nullptr)
{
    // 120 Hz substeps via 60 Hz update slices, like the app.
    const int frames = int(seconds * 60.0f);
    for (int i = 0; i < frames; ++i)
        w.Update(1.0f / 60.0f, jobs);
}

static uint32_t SleepingCount(const PhysicsWorld& w)
{
    uint32_t n = 0;
    for (uint32_t i = 0; i < w.BodyCount(); ++i)
        if (w.Body(i).Sleeping) ++n;
    return n;
}

static void BuildStack(PhysicsWorld& w, float x, int boxes)
{
    for (int i = 0; i < boxes; ++i)
        w.AddBox(Vec4(x, 0.5f + 1.0f * float(i), 0.0f, 0.0f), Quat(),
                 Vec4(0.5f, 0.5f, 0.5f, 0.0f), 1.0f);
}

int main()
{
    // 1. A settled stack goes to sleep, and its pose stays frozen afterwards.
    {
        PhysicsWorld w;
        w.AddStaticPlane(Vec4(0, 1, 0, 0), 0.0f);
        BuildStack(w, 0.0f, 5);
        StepSeconds(w, 3.0f);
        Check("stack of 5 sleeps within 3s", SleepingCount(w) == 5);

        // Positions must be bit-identical across further stepping while asleep.
        std::vector<Vec4> before(w.BodyCount());
        for (uint32_t i = 0; i < w.BodyCount(); ++i) before[i] = w.Body(i).Position;
        StepSeconds(w, 1.0f);
        bool frozen = true;
        for (uint32_t i = 0; i < w.BodyCount(); ++i)
            frozen = frozen && std::memcmp(&before[i], &w.Body(i).Position, sizeof(Vec4)) == 0;
        Check("sleeping stack is bit-frozen", frozen && SleepingCount(w) == 5);
    }

    // 2. Island independence: disturbing one stack must not wake the other.
    {
        PhysicsWorld w;
        w.AddStaticPlane(Vec4(0, 1, 0, 0), 0.0f);
        BuildStack(w, -4.0f, 4);   // bodies 1..4
        BuildStack(w,  4.0f, 4);   // bodies 5..8
        StepSeconds(w, 3.0f);
        Check("two separate stacks both sleep", SleepingCount(w) == 8);

        // Drop a box onto the LEFT stack only. (A sphere would be wrong here:
        // the engine has no rolling resistance, so spheres roll indefinitely
        // and legitimately never sleep — boxes settle.)
        w.AddBox(Vec4(-4.0f, 8.0f, 0.0f, 0.0f), Quat(),
                 Vec4(0.4f, 0.4f, 0.4f, 0.0f), 1.0f);
        StepSeconds(w, 1.0f);
        bool rightAsleep = true;
        for (uint32_t i = 5; i <= 8; ++i) rightAsleep = rightAsleep && w.Body(i).Sleeping;
        Check("impact wakes only the hit island", rightAsleep);

        // And eventually everything settles asleep again (incl. the new box).
        StepSeconds(w, 5.0f);
        Check("disturbed island re-sleeps", SleepingCount(w) == w.BodyCount() - 1); // -1 static plane
    }

    // 3. Waking actually resumes simulation: flip gravity upward after sleep —
    //    WakeAll + steps must move the boxes (they'd stay frozen otherwise).
    {
        PhysicsWorld w;
        w.AddStaticPlane(Vec4(0, 1, 0, 0), 0.0f);
        BuildStack(w, 0.0f, 3);
        StepSeconds(w, 3.0f);
        Check("stack asleep before gravity flip", SleepingCount(w) == 3);

        w.Gravity = Vec4(0.0f, 4.0f, 0.0f, 0.0f);
        w.WakeAll();
        StepSeconds(w, 1.0f);
        bool rising = true;
        for (uint32_t i = 1; i <= 3; ++i) {
            float p[4]; w.Body(i).Position.Store(p);
            rising = rising && p[1] > 1.0f + 1.0f * float(i - 1);   // above start height
        }
        Check("WakeAll + inverted gravity lifts the stack", rising);
    }

    // 4. EnableSleeping = false: nothing ever sleeps.
    {
        PhysicsWorld w;
        w.EnableSleeping = false;
        w.AddStaticPlane(Vec4(0, 1, 0, 0), 0.0f);
        BuildStack(w, 0.0f, 4);
        StepSeconds(w, 3.0f);
        Check("sleeping disabled: all bodies stay awake", SleepingCount(w) == 0);
    }

    // 4b. Split impulse: a deeply overlapping spawn must resolve WITHOUT
    //     launching. Baumgarte turns 0.2m of penetration into a ~4-5 m/s bias
    //     velocity (the classic "pop"); the split-impulse position pass
    //     displaces the body instead. Both must end at the correct rest height.
    {
        auto peakUpwardVel = [](bool split, float& outRestY) {
            PhysicsWorld w;
            w.SolverParams.SplitImpulse = split;
            w.EnableSleeping = false;                  // measure raw dynamics
            w.AddStaticPlane(Vec4(0, 1, 0, 0), 0.0f);
            // Half-extent 0.5 box with its center at 0.3 = 0.2m into the floor.
            w.AddBox(Vec4(0.0f, 0.3f, 0.0f, 0.0f), Quat(),
                     Vec4(0.5f, 0.5f, 0.5f, 0.0f), 1.0f);
            float peak = 0.0f;
            for (int f = 0; f < 120; ++f) {
                w.Update(1.0f / 60.0f, nullptr);
                float v[4]; w.Body(1).LinearVelocity.Store(v);
                peak = std::max(peak, v[1]);
            }
            float p[4]; w.Body(1).Position.Store(p);
            outRestY = p[1];
            return peak;
        };

        float restSplit = 0.0f, restBaum = 0.0f;
        const float peakSplit = peakUpwardVel(true, restSplit);
        const float peakBaum  = peakUpwardVel(false, restBaum);
        std::printf("    -> peak upward vel: split %.3f m/s, baumgarte %.3f m/s\n",
                    peakSplit, peakBaum);
        Check("split impulse resolves deep overlap without launching",
              peakSplit < 0.5f && std::fabs(restSplit - 0.5f) < 0.02f);
        Check("baumgarte injects visibly more energy (the A/B point)",
              peakBaum > 2.0f * std::max(peakSplit, 0.05f));
    }

    // 4c. Capsules: rest heights against plane and box, and capsule-on-capsule
    //     support. halfLen 0.5, radius 0.3 throughout.
    {
        const Quat lieX = Quat::FromAxisAngle(Vec4(0, 0, 1, 0), 1.5707963f); // axis -> world X
        const Quat lieZ = Quat::FromAxisAngle(Vec4(1, 0, 0, 0), 1.5707963f); // axis -> world Z

        // Lying capsule on the plane: rests at y = radius (two cap contacts).
        PhysicsWorld w;
        w.AddStaticPlane(Vec4(0, 1, 0, 0), 0.0f);
        w.AddCapsule(Vec4(0, 2.0f, 0, 0), lieX, 0.5f, 0.3f, 1.0f);
        StepSeconds(w, 2.5f);
        float p[4]; w.Body(1).Position.Store(p);
        Check("capsule lies on the plane at y = r (and sleeps)",
              std::fabs(p[1] - 0.3f) < 0.02f && w.Body(1).Sleeping);

        // Crossed capsule dropped onto it: rests in the saddle at y = 3r.
        w.AddCapsule(Vec4(0, 1.5f, 0, 0), lieZ, 0.5f, 0.3f, 1.0f);
        StepSeconds(w, 3.0f);
        float q[4]; w.Body(2).Position.Store(q);
        Check("crossed capsule rests on the first at y = 3r",
              std::fabs(q[1] - 0.9f) < 0.05f);

        // Lying capsule on a STATIC box: rests at boxTop + r.
        PhysicsWorld w2;
        w2.AddStaticPlane(Vec4(0, 1, 0, 0), 0.0f);
        w2.AddBox(Vec4(0, 0.5f, 0, 0), Quat(), Vec4(1.0f, 0.5f, 1.0f, 0), 0.0f); // static
        w2.AddCapsule(Vec4(0, 2.5f, 0, 0), lieX, 0.5f, 0.3f, 1.0f);
        StepSeconds(w2, 2.5f);
        float b[4]; w2.Body(2).Position.Store(b);
        Check("capsule lies on a box at y = boxTop + r (and sleeps)",
              std::fabs(b[1] - 1.3f) < 0.02f && w2.Body(2).Sleeping);
    }

    // 4d. Render-state interpolation: the accumulator's leftover fraction
    //     blends the last two substep poses; frozen bodies are exact.
    {
        PhysicsWorld w;
        w.EnableSleeping = false;
        w.AddSphere(Vec4(0, 100.0f, 0, 0), 0.5f, 1.0f);   // free fall, no contacts

        // 1.5 substeps of frame time: one substep runs, half remains.
        w.Update(1.5f / 120.0f, nullptr);
        const float alpha = w.InterpolationAlpha();
        Check("interpolation alpha reflects the leftover fraction",
              std::fabs(alpha - 0.5f) < 1e-4f);

        Vec4 rp; Quat rq;
        w.GetRenderPose(0, rp, rq);
        const Vec4 expect = Lerp(w.Body(0).PrevPosition, w.Body(0).Position, alpha);
        float e[4], g[4]; expect.Store(e); rp.Store(g);
        Check("render pose is the exact prev/current blend",
              std::fabs(e[1] - g[1]) < 1e-6f &&
              g[1] < 100.0f && g[1] > 99.0f);   // mid-fall, between the poses

        // Sleeping bodies: prev == current, so the render pose is bit-exact.
        PhysicsWorld w2;
        w2.AddStaticPlane(Vec4(0, 1, 0, 0), 0.0f);
        BuildStack(w2, 0.0f, 2);
        StepSeconds(w2, 2.5f);
        w2.Update(1.5f / 120.0f, nullptr);                 // fractional leftover
        Vec4 sp; Quat sq;
        w2.GetRenderPose(1, sp, sq);
        Check("sleeping body renders its exact frozen pose",
              w2.Body(1).Sleeping &&
              std::memcmp(&sp, &w2.Body(1).Position, sizeof(Vec4)) == 0);
    }

    // 4e. Gyroscopic term: spinning about the intermediate inertia axis must
    //     produce the Dzhanibekov flip (body-space spin sign reversals) WITH
    //     the term, none without — and the implicit solve must not add energy.
    {
        auto spinRun = [](bool gyro, int& flips, float& energyRatio) {
            PhysicsWorld w;
            w.EnableGyroscopic = gyro;
            w.EnableSleeping   = false;
            w.Gravity          = Vec4();
            w.AngularDamping   = 0.0f;   // isolate the gyroscopic behavior
            // he (0.6, 0.15, 0.3): distinct moments, intermediate axis = Z.
            w.AddBox(Vec4(0, 5, 0, 0), Quat(), Vec4(0.6f, 0.15f, 0.3f, 0), 1.0f);
            w.Body(0).AngularVelocity = Vec4(0.3f, 0.0f, 8.0f, 0.0f);

            float invD[4]; w.Body(0).InvInertiaBodyDiag.Store(invD);
            const float I[3] = { 1.0f / invD[0], 1.0f / invD[1], 1.0f / invD[2] };

            auto energy = [&]() {
                const RigidBody& b = w.Body(0);
                float wb[4];
                Transform(b.AngularVelocity, Transpose(ToMatrix(b.Orientation))).Store(wb);
                return 0.5f * (I[0] * wb[0] * wb[0] + I[1] * wb[1] * wb[1] + I[2] * wb[2] * wb[2]);
            };

            const float e0 = energy();
            float maxE = e0;
            flips = 0;
            int lastSign = 1;
            for (int f = 0; f < 600; ++f) {                  // 10 simulated seconds
                w.Update(1.0f / 60.0f, nullptr);
                const RigidBody& b = w.Body(0);
                float wb[4];
                Transform(b.AngularVelocity, Transpose(ToMatrix(b.Orientation))).Store(wb);
                if (wb[2] < -1.0f && lastSign > 0) { ++flips; lastSign = -1; }
                if (wb[2] >  1.0f && lastSign < 0) { ++flips; lastSign =  1; }
                maxE = std::max(maxE, energy());
            }
            energyRatio = maxE / e0;
        };

        int   flipsOn = 0, flipsOff = 0;
        float ratioOn = 0.0f, ratioOff = 0.0f;
        spinRun(true, flipsOn, ratioOn);
        spinRun(false, flipsOff, ratioOff);
        std::printf("    -> gyro on: %d flips, energy ratio %.4f | off: %d flips\n",
                    flipsOn, ratioOn, flipsOff);
        Check("gyroscopic term produces the Dzhanibekov flip", flipsOn >= 1);
        Check("implicit gyroscopic solve does not add energy", ratioOn < 1.05f);
        Check("no flips without the term (A/B)", flipsOff == 0);
    }

    // 5. Determinism with sleeping: serial vs JobSystem trajectories stay
    //    bit-identical through settle -> sleep -> impact -> wake -> re-sleep.
    {
        PhysicsWorld a, b;
        JobSystem jobs;
        jobs.Initialize();
        for (PhysicsWorld* w : { &a, &b }) {
            w->AddStaticPlane(Vec4(0, 1, 0, 0), 0.0f);
            BuildStack(*w, 0.0f, 5);
        }
        StepSeconds(a, 3.0f, nullptr);
        StepSeconds(b, 3.0f, &jobs);
        const Quat tilt = Quat::FromAxisAngle(Vec4(1, 0, 1, 0), 0.8f);
        for (PhysicsWorld* w : { &a, &b }) {
            w->AddSphere(Vec4(0.2f, 9.0f, 0.1f, 0.0f), 0.5f, 1.0f);
            w->AddCapsule(Vec4(-0.3f, 11.0f, 0.2f, 0.0f), tilt, 0.4f, 0.3f, 1.0f);
        }
        StepSeconds(a, 2.0f, nullptr);
        StepSeconds(b, 2.0f, &jobs);
        jobs.Shutdown();

        bool identical = a.BodyCount() == b.BodyCount();
        for (uint32_t i = 0; identical && i < a.BodyCount(); ++i) {
            identical = std::memcmp(&a.Body(i).Position, &b.Body(i).Position, sizeof(Vec4)) == 0
                     && std::memcmp(&a.Body(i).Orientation, &b.Body(i).Orientation, sizeof(Quat)) == 0
                     && a.Body(i).Sleeping == b.Body(i).Sleeping;
        }
        Check("serial vs JobSystem bit-identical through sleep/wake", identical);
    }

    std::printf("\n%s\n", g_failures == 0 ? "ALL PASS" : "FAILURES");
    return g_failures;
}
