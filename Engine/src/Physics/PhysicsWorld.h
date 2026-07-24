#pragma once
// ---------------------------------------------------------------------------
// PhysicsWorld — the hand-written rigid-body simulation, self-contained and
// ECS-independent. Owns one dense array of RigidBody (handles are indices;
// append-only / bulk-Clear, so they never dangle) plus joints, the manifold
// cache and all per-step scratch.
//
// Update() consumes wall-clock time through a fixed-timestep accumulator
// (FixedHz substeps, at most 4 per frame — excess time is dropped rather than
// spiraling). Each substep:
//
//   integrate forces + refresh I_w^-1   (parallel)
//   broadphase AABBs                    (parallel)  -> grid pairs   (serial)
//   narrowphase per pair                (parallel, per-index slots)
//   warm start via manifold cache       (serial)
//   solve joints + contacts, N iters    (serial)
//   integrate positions                 (parallel)
//   joint position projection (NGS)     (serial)
//
// Threading model: parallel stages write ONLY their own index's slot, the
// solver stays serial, and the pair list is sorted — so the trajectory is
// bit-identical with or without a JobSystem (the demo scene proves it with a
// live toggle). Pass jobs == nullptr and the same code runs as inline loops.
// ---------------------------------------------------------------------------
#include "Broadphase.h"
#include "Contact.h"
#include "ContactSolver.h"
#include "Joint.h"
#include "RigidBody.h"

#include <cstdint>
#include <vector>

namespace SGE { class JobSystem; }

namespace SGE::Physics {

class PhysicsWorld {
public:
    struct StepTimings {
        float    IntegrateMs = 0.0f;   // force + position integration, inertia refresh
        float    BroadMs     = 0.0f;   // AABBs + grid build + pair collection
        float    NarrowMs    = 0.0f;   // exact contact generation
        float    SolveMs     = 0.0f;   // warm start + all solver iterations
        float    TotalMs     = 0.0f;   // whole Update() call
        uint32_t Substeps    = 0;
        uint32_t Pairs       = 0;      // candidate pairs (last substep)
        uint32_t Contacts    = 0;      // contact points   (last substep)
    };

    // --- construction (mass <= 0 makes the body static) ---
    BodyHandle AddSphere(Math::Vec4 pos, float radius, float mass);
    BodyHandle AddBox(Math::Vec4 pos, Math::Quat orientation, Math::Vec4 halfExtents, float mass);
    BodyHandle AddStaticPlane(Math::Vec4 normal, float d);

    // Anchors/axes are given in WORLD space at the current pose.
    void AddDistanceJoint(BodyHandle a, BodyHandle b, Math::Vec4 worldAnchorA, Math::Vec4 worldAnchorB);
    void AddBallJoint(BodyHandle a, BodyHandle b, Math::Vec4 worldAnchor);
    void AddHingeJoint(BodyHandle a, BodyHandle b, Math::Vec4 worldAnchor, Math::Vec4 worldAxis);

    void Clear();

    // --- stepping ---
    void Update(float frameDt, SGE::JobSystem* jobs = nullptr);

    // --- access ---
    RigidBody&       Body(BodyHandle h)       { return m_bodies[h]; }
    const RigidBody& Body(BodyHandle h) const { return m_bodies[h]; }
    uint32_t         BodyCount()        const { return uint32_t(m_bodies.size()); }
    uint32_t         JointCount()       const {
        return uint32_t(m_distanceJoints.size() + m_ballJoints.size() + m_hingeJoints.size());
    }
    const std::vector<Manifold>& Manifolds() const { return m_manifolds; } // last substep (debug overlay)
    const StepTimings&           Timings()   const { return m_timings; }

    // --- tunables (live-editable from the demo panels) ---
    Math::Vec4 Gravity { 0.0f, -9.81f, 0.0f, 0.0f };
    int        Iterations = 10;      // velocity iterations per substep
    float      FixedHz    = 120.0f;
    float      CellSize   = 2.0f;    // broadphase grid cell size
    bool       UseGrid    = true;    // false = brute-force O(n^2) broadphase (A/B + validation)
    ContactSolver::Params SolverParams;

    // First-order integration with no gyroscopic term lets long-running
    // undamped rigs (a frictionless whirling hinge) slowly gain rotational
    // energy. Industry-standard guards, both tunable: a touch of angular
    // damping (Unity ships 0.05 as its default) and a hard angular speed cap
    // so no body rotates unphysically far within one substep.
    float LinearDamping   = 0.0f;    // 1/s
    float AngularDamping  = 0.05f;   // 1/s
    float MaxAngularSpeed = 30.0f;   // rad/s (~0.25 rad per 120 Hz substep)

private:
    void Step(float h, SGE::JobSystem* jobs);

    std::vector<RigidBody>     m_bodies;
    std::vector<DistanceJoint> m_distanceJoints;
    std::vector<BallJoint>     m_ballJoints;
    std::vector<HingeJoint>    m_hingeJoints;

    Broadphase    m_broadphase;
    ManifoldCache m_cache;
    float         m_accumulator = 0.0f;
    StepTimings   m_timings;

    // Per-step scratch, kept as members so steady-state steps don't allocate.
    std::vector<AABB>     m_aabbs;
    std::vector<uint32_t> m_gridIndices;   // non-plane bodies
    std::vector<uint32_t> m_planes;        // plane bodies
    std::vector<uint64_t> m_pairs;         // packed (lo << 32 | hi)
    std::vector<Manifold> m_pairManifolds; // one slot per pair (parallel narrowphase)
    std::vector<Manifold> m_manifolds;     // compacted, in pair order
};

} // namespace SGE::Physics
