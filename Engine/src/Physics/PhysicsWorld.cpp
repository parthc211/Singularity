#include "Physics/PhysicsWorld.h"
#include "Physics/Narrowphase.h"
#include "Jobs/JobSystem.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>

namespace SGE::Physics {

using namespace SGE::Math;

namespace {

using clock_t_ = std::chrono::steady_clock;

float MsSince(clock_t_::time_point t0) {
    return std::chrono::duration<float, std::milli>(clock_t_::now() - t0).count();
}

// One code path for both threading modes: with a JobSystem the range fans out
// across workers (each index writes only its own slot — see the header's
// threading model), without one it degrades to a plain loop.
template <typename Fn>
void ParallelFor(SGE::JobSystem* jobs, uint32_t count, uint32_t group, Fn&& fn) {
    if (count == 0) return;
    if (jobs) {
        jobs->Dispatch(count, group, fn);
        jobs->Wait();
    } else {
        for (uint32_t i = 0; i < count; ++i) fn(i);
    }
}

} // namespace

// ============================= construction ================================

BodyHandle PhysicsWorld::AddSphere(Vec4 pos, float radius, float mass) {
    RigidBody b{};
    b.Position     = pos;
    b.Shape.Type   = ShapeType::Sphere;
    b.Shape.Radius = radius;
    if (mass > 0.0f) {
        b.InvMass = 1.0f / mass;
        const float invI = 1.0f / (0.4f * mass * radius * radius); // I = 2/5 m r^2
        b.InvInertiaBodyDiag = Vec4(invI, invI, invI, 0.0f);
    }
    m_bodies.push_back(b);
    return BodyHandle(m_bodies.size() - 1);
}

BodyHandle PhysicsWorld::AddBox(Vec4 pos, Quat orientation, Vec4 halfExtents, float mass) {
    RigidBody b{};
    b.Position      = pos;
    b.Orientation   = orientation;
    b.Shape.Type    = ShapeType::Box;
    b.Shape.Extents = halfExtents;
    if (mass > 0.0f) {
        b.InvMass = 1.0f / mass;
        float he[4]; halfExtents.Store(he);
        // Solid cuboid: I_x = m/12 ((2hy)^2 + (2hz)^2) = m/3 (hy^2 + hz^2), etc.
        const float k = mass / 3.0f;
        b.InvInertiaBodyDiag = Vec4(1.0f / (k * (he[1]*he[1] + he[2]*he[2])),
                                    1.0f / (k * (he[0]*he[0] + he[2]*he[2])),
                                    1.0f / (k * (he[0]*he[0] + he[1]*he[1])), 0.0f);
    }
    m_bodies.push_back(b);
    return BodyHandle(m_bodies.size() - 1);
}

BodyHandle PhysicsWorld::AddStaticPlane(Vec4 normal, float d) {
    RigidBody b{};
    Vec4 n = Normalize3(normal);
    b.Shape.Type    = ShapeType::Plane;
    b.Shape.Extents = Vec4(n.x(), n.y(), n.z(), d);
    b.Friction      = 0.8f;
    b.Restitution   = 0.0f;
    m_bodies.push_back(b);
    return BodyHandle(m_bodies.size() - 1);
}

void PhysicsWorld::AddDistanceJoint(BodyHandle a, BodyHandle b, Vec4 worldAnchorA, Vec4 worldAnchorB) {
    DistanceJoint j{};
    j.A = a; j.B = b;
    j.LocalAnchorA = Rotate(Conjugate(m_bodies[a].Orientation), worldAnchorA - m_bodies[a].Position);
    j.LocalAnchorB = Rotate(Conjugate(m_bodies[b].Orientation), worldAnchorB - m_bodies[b].Position);
    j.Length       = Length3(worldAnchorB - worldAnchorA);
    m_distanceJoints.push_back(j);
}

void PhysicsWorld::AddBallJoint(BodyHandle a, BodyHandle b, Vec4 worldAnchor) {
    BallJoint j{};
    j.A = a; j.B = b;
    j.LocalAnchorA = Rotate(Conjugate(m_bodies[a].Orientation), worldAnchor - m_bodies[a].Position);
    j.LocalAnchorB = Rotate(Conjugate(m_bodies[b].Orientation), worldAnchor - m_bodies[b].Position);
    m_ballJoints.push_back(j);
}

void PhysicsWorld::AddHingeJoint(BodyHandle a, BodyHandle b, Vec4 worldAnchor, Vec4 worldAxis) {
    HingeJoint j{};
    j.A = a; j.B = b;
    j.LocalAnchorA = Rotate(Conjugate(m_bodies[a].Orientation), worldAnchor - m_bodies[a].Position);
    j.LocalAnchorB = Rotate(Conjugate(m_bodies[b].Orientation), worldAnchor - m_bodies[b].Position);

    Vec4 axis = Normalize3(worldAxis);
    j.LocalAxisA = Rotate(Conjugate(m_bodies[a].Orientation), axis);
    // Freeze a perpendicular basis in B's frame; the solver keeps A's axis
    // perpendicular to both, which pins the two rotational DOF off the hinge.
    Vec4 refAxis = std::fabs(axis.x()) < 0.57f ? Vec4(1, 0, 0, 0) : Vec4(0, 1, 0, 0);
    Vec4 p1 = Normalize3(Cross(axis, refAxis));
    Vec4 p2 = Cross(axis, p1);
    j.LocalB2 = Rotate(Conjugate(m_bodies[b].Orientation), p1);
    j.LocalC2 = Rotate(Conjugate(m_bodies[b].Orientation), p2);
    m_hingeJoints.push_back(j);
}

void PhysicsWorld::Clear() {
    m_bodies.clear();
    m_distanceJoints.clear();
    m_ballJoints.clear();
    m_hingeJoints.clear();
    m_cache.clear();
    m_manifolds.clear();
    m_accumulator = 0.0f;
    m_timings = {};
}

// =============================== stepping ==================================

void PhysicsWorld::Update(float frameDt, SGE::JobSystem* jobs) {
    const auto t0 = clock_t_::now();
    m_timings = {};

    const float h = 1.0f / FixedHz;
    m_accumulator += std::min(frameDt, 0.25f);
    // At most 4 substeps per frame; excess time is dropped so a slow frame
    // can't snowball into ever more physics work (no death spiral).
    m_accumulator = std::min(m_accumulator, 4.0f * h);

    while (m_accumulator >= h) {
        Step(h, jobs);
        m_accumulator -= h;
        ++m_timings.Substeps;
    }
    m_timings.TotalMs = MsSince(t0);
}

void PhysicsWorld::Step(float h, SGE::JobSystem* jobs) {
    const uint32_t n    = uint32_t(m_bodies.size());
    const float    invH = 1.0f / h;

    // --- 1. integrate forces + refresh world-space inverse inertia (parallel)
    auto t = clock_t_::now();
    ParallelFor(jobs, n, 64, [this, h](uint32_t i) {
        RigidBody& b = m_bodies[i];
        if (!b.IsStatic()) {
            b.LinearVelocity += Gravity * h;
            // Implicit exponential damping: v / (1 + h*d) is unconditionally
            // stable for any d, unlike the explicit v * (1 - h*d).
            b.LinearVelocity  = b.LinearVelocity  * (1.0f / (1.0f + h * LinearDamping));
            b.AngularVelocity = b.AngularVelocity * (1.0f / (1.0f + h * AngularDamping));
            const float wSq = LengthSq3(b.AngularVelocity);
            if (wSq > MaxAngularSpeed * MaxAngularSpeed)
                b.AngularVelocity = b.AngularVelocity * (MaxAngularSpeed / std::sqrt(wSq));
        }
        Mat4 R = ToMatrix(b.Orientation);
        float d[4]; b.InvInertiaBodyDiag.Store(d);
        // Row-vector convention: I_w^-1 = R^T diag(I_b^-1) R (validated in MathBenchmark).
        b.InvInertiaWorld = Mul(Mul(Transpose(R), Mat4::Scale(d[0], d[1], d[2])), R);
    });
    m_timings.IntegrateMs += MsSince(t);

    // --- 2. broadphase: AABBs (parallel), then pair collection (serial) ---
    t = clock_t_::now();
    m_aabbs.resize(n);
    m_gridIndices.clear();
    m_planes.clear();
    for (uint32_t i = 0; i < n; ++i)
        (m_bodies[i].Shape.Type == ShapeType::Plane ? m_planes : m_gridIndices).push_back(i);

    ParallelFor(jobs, uint32_t(m_gridIndices.size()), 64, [this](uint32_t k) {
        const uint32_t i = m_gridIndices[k];
        m_aabbs[i] = ComputeAABB(m_bodies[i]);
    });

    m_pairs.clear();
    if (UseGrid) {
        m_broadphase.CellSize = CellSize;
        m_broadphase.CollectPairs(m_bodies, m_aabbs, m_gridIndices, m_pairs);
    } else {
        for (size_t p = 0; p + 1 < m_gridIndices.size(); ++p)
            for (size_t q = p + 1; q < m_gridIndices.size(); ++q) {
                const uint32_t i = m_gridIndices[p], j = m_gridIndices[q];
                if (m_bodies[i].IsStatic() && m_bodies[j].IsStatic()) continue;
                if (!AABBOverlap(m_aabbs[i], m_aabbs[j])) continue;
                m_pairs.push_back((uint64_t(std::min(i, j)) << 32) | std::max(i, j));
            }
        std::sort(m_pairs.begin(), m_pairs.end());
    }
    // Planes are not in the grid: test every dynamic body against every plane.
    for (uint32_t p : m_planes)
        for (uint32_t i : m_gridIndices)
            if (!m_bodies[i].IsStatic())
                m_pairs.push_back((uint64_t(i) << 32) | p);
    m_timings.BroadMs += MsSince(t);

    // --- 3. narrowphase (parallel; each pair writes only its own slot) ---
    t = clock_t_::now();
    m_pairManifolds.assign(m_pairs.size(), Manifold{});
    ParallelFor(jobs, uint32_t(m_pairs.size()), 16, [this](uint32_t k) {
        uint32_t x = uint32_t(m_pairs[k] >> 32);
        uint32_t y = uint32_t(m_pairs[k] & 0xFFFFFFFFu);
        // Canonical order for dispatch: lower ShapeType first (index order
        // breaks ties — already guaranteed by the packing).
        if (m_bodies[x].Shape.Type > m_bodies[y].Shape.Type) std::swap(x, y);
        Manifold& m = m_pairManifolds[k];
        m.A = x;
        m.B = y;
        if (!Narrowphase::Collide(m_bodies[x], m_bodies[y], m)) m.Count = 0;
    });
    // Serial compaction in pair order keeps solve order deterministic.
    m_manifolds.clear();
    for (const Manifold& m : m_pairManifolds)
        if (m.Count > 0) m_manifolds.push_back(m);
    m_timings.NarrowMs += MsSince(t);
    m_timings.Pairs    = uint32_t(m_pairs.size());
    m_timings.Contacts = 0;
    for (const Manifold& m : m_manifolds) m_timings.Contacts += m.Count;

    // --- 4. warm start: adopt last substep's impulses, presolve ---
    // Matching is by FeatureId first, then by nearest position. The fallback
    // matters: when a stacked box tilts by microns, the clipper can mint
    // different vertex ids for what is physically the same contact — dropping
    // those accumulated impulses exactly when the load is highest is what
    // makes tall stacks explode.
    t = clock_t_::now();
    constexpr float kMatchDistSq = 0.02f * 0.02f;
    for (Manifold& m : m_manifolds) {
        if (auto it = m_cache.find(PairKey(m.A, m.B)); it != m_cache.end()) {
            const Manifold& old = it->second;
            bool usedOld[4] = {}, matched[4] = {};
            for (uint32_t i = 0; i < m.Count; ++i)
                for (uint32_t k = 0; k < old.Count; ++k)
                    if (!usedOld[k] && m.Points[i].FeatureId == old.Points[k].FeatureId) {
                        m.Points[i].NormalImpulse     = old.Points[k].NormalImpulse;
                        m.Points[i].TangentImpulse[0] = old.Points[k].TangentImpulse[0];
                        m.Points[i].TangentImpulse[1] = old.Points[k].TangentImpulse[1];
                        usedOld[k] = matched[i] = true;
                        break;
                    }
            for (uint32_t i = 0; i < m.Count; ++i) {
                if (matched[i]) continue;
                float best = kMatchDistSq;
                int   bestK = -1;
                for (uint32_t k = 0; k < old.Count; ++k) {
                    if (usedOld[k]) continue;
                    const float dsq = Math::LengthSq3(m.Points[i].Position - old.Points[k].Position);
                    if (dsq < best) { best = dsq; bestK = int(k); }
                }
                if (bestK >= 0) {
                    m.Points[i].NormalImpulse     = old.Points[bestK].NormalImpulse;
                    m.Points[i].TangentImpulse[0] = old.Points[bestK].TangentImpulse[0];
                    m.Points[i].TangentImpulse[1] = old.Points[bestK].TangentImpulse[1];
                    usedOld[bestK] = true;
                }
            }
        }
        ContactSolver::Presolve(m_bodies, m, invH, SolverParams);
    }
    for (auto& j : m_distanceJoints) Joints::Presolve(m_bodies, j);
    for (auto& j : m_ballJoints)     Joints::Presolve(m_bodies, j);
    for (auto& j : m_hingeJoints)    Joints::Presolve(m_bodies, j);

    // Warm starting is a SEPARATE pass after every presolve: presolve samples
    // the approach speed that decides restitution, and warm-start impulses
    // (often an entire stack's weight) make intermediate velocities unphysical
    // until the iterations rebalance — interleaving the two passes reads that
    // garbage as real impacts and pumps energy into resting stacks.
    for (Manifold& m : m_manifolds)  ContactSolver::WarmStart(m_bodies, m);
    for (auto& j : m_distanceJoints) Joints::WarmStart(m_bodies, j);
    for (auto& j : m_ballJoints)     Joints::WarmStart(m_bodies, j);
    for (auto& j : m_hingeJoints)    Joints::WarmStart(m_bodies, j);

    // --- 5. sequential-impulse iterations: joints first, then contacts ---
    for (int it = 0; it < Iterations; ++it) {
        for (auto& j : m_distanceJoints) Joints::Solve(m_bodies, j);
        for (auto& j : m_ballJoints)     Joints::Solve(m_bodies, j);
        for (auto& j : m_hingeJoints)    Joints::Solve(m_bodies, j);
        for (Manifold& m : m_manifolds)  ContactSolver::SolveIteration(m_bodies, m);
    }

    // Persist accumulated impulses for next substep's warm start.
    m_cache.clear();
    for (const Manifold& m : m_manifolds)
        m_cache.emplace(PairKey(m.A, m.B), m);
    m_timings.SolveMs += MsSince(t);

    // --- 6. integrate positions (parallel) ---
    t = clock_t_::now();
    ParallelFor(jobs, n, 64, [this, h](uint32_t i) {
        RigidBody& b = m_bodies[i];
        if (b.IsStatic()) return;
        b.Position += b.LinearVelocity * h;
        // q' = Normalize(q + (w_quat * q) * h/2) — first-order quat integration.
        Vec4 w = b.AngularVelocity;
        Quat wq(w.x(), w.y(), w.z(), 0.0f);
        b.Orientation = Normalize(b.Orientation + Mul(wq, b.Orientation) * (0.5f * h));
#ifdef SGE_DEBUG
        float pf[4]; b.Position.Store(pf);
        assert(std::isfinite(pf[0]) && std::isfinite(pf[1]) && std::isfinite(pf[2])
               && "PhysicsWorld: body position went non-finite");
#endif
    });
    m_timings.IntegrateMs += MsSince(t);

    // --- 7. joint position projection (NGS) ---
    // Joints correct positional drift here, AFTER integration, by moving
    // bodies directly — no velocity change, so no injected kinetic energy
    // (the velocity rows above run bias-free; see Joint.h).
    if (JointCount() > 0) {
        t = clock_t_::now();
        for (int it = 0; it < 3; ++it) {
            for (auto& j : m_distanceJoints) Joints::SolvePosition(m_bodies, j);
            for (auto& j : m_ballJoints)     Joints::SolvePosition(m_bodies, j);
            for (auto& j : m_hingeJoints)    Joints::SolvePosition(m_bodies, j);
        }
        m_timings.SolveMs += MsSince(t);
    }
}

} // namespace SGE::Physics
