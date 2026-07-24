#pragma once
// ---------------------------------------------------------------------------
// Sequential-impulse contact solver (the Box2D recipe, in 3D).
//
// Per substep: Presolve computes each contact row's effective mass, friction
// basis and bias velocity; WarmStart re-applies the impulses accumulated last
// substep (matched by FeatureId before this is called); then SolveIteration
// runs Gauss-Seidel style N times, clamping the ACCUMULATED impulse — not the
// per-iteration delta — which is the trick that lets impulses be provisionally
// over-applied and corrected, and is why stacks converge.
//
// Normal rows clamp to lambda >= 0 (contacts push, never pull). Friction rows
// clamp to the cone |lambda_t| <= mu * lambda_n using the current accumulated
// normal impulse. Position error feeds back as a Baumgarte bias velocity
// (beta/h * max(pen - slop, 0)); restitution replaces it when the approach
// speed is above a threshold, using max() so the two never pump together.
// ---------------------------------------------------------------------------
#include "Contact.h"
#include <vector>

namespace SGE::Physics::ContactSolver {

struct Params {
    float Beta                 = 0.2f;      // Baumgarte position-correction factor
    float Slop                 = 0.005f;    // penetration tolerated without correction (m)
    float RestitutionThreshold = 1.0f;      // approach speed (m/s) below which e = 0
};

void Presolve(std::vector<RigidBody>& bodies, Manifold& m, float invH, const Params& p);
void WarmStart(std::vector<RigidBody>& bodies, Manifold& m);
void SolveIteration(std::vector<RigidBody>& bodies, Manifold& m);

} // namespace SGE::Physics::ContactSolver
