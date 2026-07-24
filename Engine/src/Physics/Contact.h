#pragma once
// ---------------------------------------------------------------------------
// Contact manifolds — the interface between narrowphase and the solver.
//
// A Manifold is up to 4 contact points shared by one body pair, with one
// normal (unit, pointing from body A toward body B). Narrowphase fills
// Position/Penetration/FeatureId; ContactSolver::Presolve derives the rest
// (lever arms, effective masses, bias velocities).
//
// PERSISTENCE is what makes stacks possible: manifolds are cached across
// substeps keyed by the body pair, and each point carries a FeatureId naming
// the geometric feature it came from (box corner index, clip vertex id, ...).
// When the same feature shows up next substep, the previously accumulated
// impulses are copied over and re-applied ("warm starting"), so the solver
// starts each substep already near the converged answer instead of from zero.
// Without this, 10 iterations is nowhere near enough for a 10-box stack.
// ---------------------------------------------------------------------------
#include "RigidBody.h"
#include <unordered_map>
#include <cstdint>

namespace SGE::Physics {

struct alignas(16) ContactPoint {
    Math::Vec4 Position;                    // world-space contact point
    Math::Vec4 rA, rB;                      // world offsets from each body's center (presolve)
    float      Penetration = 0.0f;          // > 0 means overlapping
    uint32_t   FeatureId   = 0;             // warm-start matching key

    // Accumulated impulses (the values warm starting carries across substeps).
    float NormalImpulse     = 0.0f;
    float TangentImpulse[2] = { 0.0f, 0.0f };

    // Presolve caches.
    float MassNormal     = 0.0f;            // 1 / (J M^-1 J^T) for the normal row
    float MassTangent[2] = { 0.0f, 0.0f };
    float Bias           = 0.0f;            // Baumgarte + restitution target velocity
};

struct alignas(16) Manifold {
    Math::Vec4   Normal;                    // unit, from A to B
    Math::Vec4   Tangent[2];                // orthonormal friction basis (presolve)
    ContactPoint Points[4];
    BodyHandle   A = 0, B = 0;
    uint32_t     Count = 0;
    float        Friction = 0.0f;           // combined (presolve)
};

// Persistent store, keyed by (min(A,B) << 32) | max(A,B). Rebuilt every
// substep from the fresh narrowphase results; old entries donate their
// accumulated impulses to matching FeatureIds first.
using ManifoldCache = std::unordered_map<uint64_t, Manifold>;

inline uint64_t PairKey(BodyHandle a, BodyHandle b) {
    const uint64_t lo = a < b ? a : b;
    const uint64_t hi = a < b ? b : a;
    return (lo << 32) | hi;
}

} // namespace SGE::Physics
