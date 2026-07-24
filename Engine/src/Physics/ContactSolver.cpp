#include "Physics/ContactSolver.h"

#include <algorithm>
#include <cmath>

namespace SGE::Physics::ContactSolver {

using namespace SGE::Math;

namespace {

// Effective mass of a 1D row along `dir` through lever arms rA/rB:
//   k = 1/mA + 1/mB + (rA x d).Ia^-1(rA x d) + (rB x d).Ib^-1(rB x d)
// (the scalar J M^-1 J^T of this row; valid because I^-1 is symmetric).
float EffectiveMass(const RigidBody& a, const RigidBody& b, Vec4 rA, Vec4 rB, Vec4 dir) {
    Vec4 raxd = Cross(rA, dir);
    Vec4 rbxd = Cross(rB, dir);
    const float k = a.InvMass + b.InvMass
                  + Dot3(raxd, Transform(raxd, a.InvInertiaWorld))
                  + Dot3(rbxd, Transform(rbxd, b.InvInertiaWorld));
    return k > 0.0f ? 1.0f / k : 0.0f;
}

// Velocity of the material point at offset r from the body center.
Vec4 VelocityAt(const RigidBody& body, Vec4 r) {
    return body.LinearVelocity + Cross(body.AngularVelocity, r);
}

// Apply impulse P at the contact: equal and opposite, torque via the lever arms.
void ApplyImpulse(RigidBody& a, RigidBody& b, Vec4 rA, Vec4 rB, Vec4 P) {
    a.LinearVelocity  -= P * a.InvMass;
    a.AngularVelocity -= Transform(Cross(rA, P), a.InvInertiaWorld);
    b.LinearVelocity  += P * b.InvMass;
    b.AngularVelocity += Transform(Cross(rB, P), b.InvInertiaWorld);
}

} // namespace

void Presolve(std::vector<RigidBody>& bodies, Manifold& m, float invH, const Params& p) {
    RigidBody& a = bodies[m.A];
    RigidBody& b = bodies[m.B];

    m.Friction = std::sqrt(a.Friction * b.Friction);
    const float e = std::max(a.Restitution, b.Restitution);

    // Friction basis: t0 from the world axis least aligned with the normal,
    // t1 completes the right-handed frame. Recomputed every substep; the
    // warm-started tangent scalars carry over (standard Box2D behavior — the
    // basis drifts slowly relative to a substep).
    const Vec4 n = m.Normal;
    const Vec4 refAxis = std::fabs(n.x()) < 0.57f ? Vec4(1, 0, 0, 0) : Vec4(0, 1, 0, 0);
    m.Tangent[0] = Normalize3(Cross(n, refAxis));
    m.Tangent[1] = Cross(n, m.Tangent[0]);          // unit by construction

    for (uint32_t i = 0; i < m.Count; ++i) {
        ContactPoint& cp = m.Points[i];
        cp.rA = cp.Position - a.Position;
        cp.rB = cp.Position - b.Position;

        cp.MassNormal     = EffectiveMass(a, b, cp.rA, cp.rB, n);
        cp.MassTangent[0] = EffectiveMass(a, b, cp.rA, cp.rB, m.Tangent[0]);
        cp.MassTangent[1] = EffectiveMass(a, b, cp.rA, cp.rB, m.Tangent[1]);

        // Approach speed BEFORE warm starting decides restitution; max() with
        // the Baumgarte term so position correction and bounce never add up.
        // Speculative points (negative penetration = a persistence-margin gap)
        // get a NEGATIVE bias instead: the row then only clamps the approach
        // speed to gap/h — free to close the gap this substep, not to pass it.
        const float vn0  = Dot3(VelocityAt(b, cp.rB) - VelocityAt(a, cp.rA), n);
        const float rest = vn0 < -p.RestitutionThreshold ? -e * vn0 : 0.0f;
        if (cp.Penetration >= 0.0f) {
            const float baum = p.Beta * invH * std::max(cp.Penetration - p.Slop, 0.0f);
            cp.Bias = std::max(baum, rest);
        } else {
            cp.Bias = std::max(cp.Penetration * invH, rest);
            // A speculative point has no opposing surface yet — re-applying a
            // cached impulse across the gap would fire the bodies apart (the
            // cached value can be an entire stack's weight). It keeps its
            // velocity clamp but starts the impulse accumulation from zero.
            cp.NormalImpulse     = 0.0f;
            cp.TangentImpulse[0] = 0.0f;
            cp.TangentImpulse[1] = 0.0f;
        }
    }
}

void WarmStart(std::vector<RigidBody>& bodies, Manifold& m) {
    RigidBody& a = bodies[m.A];
    RigidBody& b = bodies[m.B];
    for (uint32_t i = 0; i < m.Count; ++i) {
        ContactPoint& cp = m.Points[i];
        Vec4 P = m.Normal * cp.NormalImpulse
               + m.Tangent[0] * cp.TangentImpulse[0]
               + m.Tangent[1] * cp.TangentImpulse[1];
        ApplyImpulse(a, b, cp.rA, cp.rB, P);
    }
}

void SolveIteration(std::vector<RigidBody>& bodies, Manifold& m) {
    RigidBody& a = bodies[m.A];
    RigidBody& b = bodies[m.B];

    for (uint32_t i = 0; i < m.Count; ++i) {
        ContactPoint& cp = m.Points[i];

        // --- normal row: accumulated clamp lambda_n >= 0 ---
        {
            const float vn  = Dot3(VelocityAt(b, cp.rB) - VelocityAt(a, cp.rA), m.Normal);
            const float dL  = cp.MassNormal * (cp.Bias - vn);
            const float old = cp.NormalImpulse;
            cp.NormalImpulse = std::max(old + dL, 0.0f);
            ApplyImpulse(a, b, cp.rA, cp.rB, m.Normal * (cp.NormalImpulse - old));
        }

        // --- friction rows: accumulated clamp to the cone mu * lambda_n ---
        const float maxT = m.Friction * cp.NormalImpulse;
        for (int t = 0; t < 2; ++t) {
            const float vt  = Dot3(VelocityAt(b, cp.rB) - VelocityAt(a, cp.rA), m.Tangent[t]);
            const float dL  = cp.MassTangent[t] * -vt;
            const float old = cp.TangentImpulse[t];
            cp.TangentImpulse[t] = std::clamp(old + dL, -maxT, maxT);
            ApplyImpulse(a, b, cp.rA, cp.rB, m.Tangent[t] * (cp.TangentImpulse[t] - old));
        }
    }
}

} // namespace SGE::Physics::ContactSolver
