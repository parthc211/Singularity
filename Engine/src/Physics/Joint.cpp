#include "Physics/Joint.h"

#include <algorithm>
#include <cmath>

namespace SGE::Physics::Joints {

using namespace SGE::Math;

namespace {

// Largest constraint error corrected per NGS iteration (m / rad). Keeps a
// deeply violated joint from teleporting bodies in one go.
constexpr float kMaxCorrection = 0.2f;

Vec4 VelocityAt(const RigidBody& body, Vec4 r) {
    return body.LinearVelocity + Cross(body.AngularVelocity, r);
}

// Point-to-point impulse P applied at lever arms rA (negative side) / rB.
void ApplyImpulse(RigidBody& a, RigidBody& b, Vec4 rA, Vec4 rB, Vec4 P) {
    a.LinearVelocity  -= P * a.InvMass;
    a.AngularVelocity -= Transform(Cross(rA, P), a.InvInertiaWorld);
    b.LinearVelocity  += P * b.InvMass;
    b.AngularVelocity += Transform(Cross(rB, P), b.InvInertiaWorld);
}

// Angular-only impulse along axis u (rows like the hinge's alignment rows).
void ApplyAngular(RigidBody& a, RigidBody& b, Vec4 u, float lambda) {
    a.AngularVelocity += Transform(u * lambda, a.InvInertiaWorld);
    b.AngularVelocity -= Transform(u * lambda, b.InvInertiaWorld);
}

// Rotate a body by a small axis-angle vector (radians) — the position-domain
// analogue of one velocity-integration step.
void RotateBy(RigidBody& b, Vec4 theta) {
    Quat tq(theta.x(), theta.y(), theta.z(), 0.0f);
    b.Orientation = Normalize(b.Orientation + Mul(tq, b.Orientation) * 0.5f);
}

// World inverse inertia from the body's CURRENT orientation. The cached
// RigidBody::InvInertiaWorld is refreshed at the start of the substep, but
// NGS runs after position integration — at high spin the orientation has
// moved several degrees since, and using the stale tensor on an anisotropic
// body turns the mass-weighted projection into an error injector.
Mat4 FreshInvInertia(const RigidBody& b) {
    Mat4 R = ToMatrix(b.Orientation);
    float d[4]; b.InvInertiaBodyDiag.Store(d);
    return Mul(Mul(Transpose(R), Mat4::Scale(d[0], d[1], d[2])), R);
}

// Position-domain twin of ApplyImpulse: displaces and rotates instead of
// changing velocities. This is what makes NGS energy-neutral.
void ApplyPositional(RigidBody& a, RigidBody& b, Vec4 rA, Vec4 rB, Vec4 P,
                     const Mat4& invIa, const Mat4& invIb) {
    a.Position -= P * a.InvMass;
    RotateBy(a, -Transform(Cross(rA, P), invIa));
    b.Position += P * b.InvMass;
    RotateBy(b, Transform(Cross(rB, P), invIb));
}

// skew(r): the matrix with  skew(r) * v = r x v  (standard entries; zero w
// row/column so 4x4 products keep the 3x3 block exact).
Mat4 Skew(Vec4 r) {
    float f[4]; r.Store(f);
    Mat4 S;
    S.r[0] = _mm_set_ps(0.0f,  f[1], -f[2],  0.0f);
    S.r[1] = _mm_set_ps(0.0f, -f[0],  0.0f,  f[2]);
    S.r[2] = _mm_set_ps(0.0f,  0.0f,  f[0], -f[1]);
    S.r[3] = _mm_setzero_ps();
    return S;
}

Mat4 Sub(const Mat4& A, const Mat4& B) {
    Mat4 C;
    for (int i = 0; i < 4; ++i) C.r[i] = _mm_sub_ps(A.r[i], B.r[i]);
    return C;
}

// K = (1/mA + 1/mB) I3 - skew(rA) Ia^-1 skew(rA) - skew(rB) Ib^-1 skew(rB).
// Symmetric positive-definite for any valid mass/inertia, so Inverse3x3 is
// safe and the row-vector/column-vector distinction is immaterial.
Mat4 BallK(const RigidBody& a, const RigidBody& b, Vec4 rA, Vec4 rB,
           const Mat4& invIa, const Mat4& invIb) {
    Mat4 K  = Mat4::Scale(a.InvMass + b.InvMass,
                          a.InvMass + b.InvMass,
                          a.InvMass + b.InvMass);
    Mat4 SA = Skew(rA);
    Mat4 SB = Skew(rB);
    K = Sub(K, Mul(SA, Mul(invIa, SA)));
    K = Sub(K, Mul(SB, Mul(invIb, SB)));
    return K;
}

Vec4 ClampLength3(Vec4 v, float maxLen) {
    const float lenSq = LengthSq3(v);
    if (lenSq > maxLen * maxLen) v = v * (maxLen / std::sqrt(lenSq));
    return v;
}

} // namespace

// ============================ DistanceJoint ================================

void Presolve(std::vector<RigidBody>& bodies, DistanceJoint& j) {
    RigidBody& a = bodies[j.A];
    RigidBody& b = bodies[j.B];

    j.rA = Rotate(a.Orientation, j.LocalAnchorA);
    j.rB = Rotate(b.Orientation, j.LocalAnchorB);

    Vec4 d = (b.Position + j.rB) - (a.Position + j.rA);
    const float len = Length3(d);
    j.Dir = len > 1e-6f ? d / len : Vec4(0, 1, 0, 0);

    Vec4 raxd = Cross(j.rA, j.Dir);
    Vec4 rbxd = Cross(j.rB, j.Dir);
    const float k = a.InvMass + b.InvMass
                  + Dot3(raxd, Transform(raxd, a.InvInertiaWorld))
                  + Dot3(rbxd, Transform(rbxd, b.InvInertiaWorld));
    j.Mass = k > 0.0f ? 1.0f / k : 0.0f;
}

void WarmStart(std::vector<RigidBody>& bodies, DistanceJoint& j) {
    ApplyImpulse(bodies[j.A], bodies[j.B], j.rA, j.rB, j.Dir * j.AccumImpulse);
}

void Solve(std::vector<RigidBody>& bodies, DistanceJoint& j) {
    RigidBody& a = bodies[j.A];
    RigidBody& b = bodies[j.B];

    const float jv     = Dot3(j.Dir, VelocityAt(b, j.rB) - VelocityAt(a, j.rA));
    const float lambda = -j.Mass * jv;              // rod: unclamped, both signs
    j.AccumImpulse += lambda;
    ApplyImpulse(a, b, j.rA, j.rB, j.Dir * lambda);
}

void SolvePosition(std::vector<RigidBody>& bodies, DistanceJoint& j) {
    RigidBody& a = bodies[j.A];
    RigidBody& b = bodies[j.B];

    Vec4 rA = Rotate(a.Orientation, j.LocalAnchorA);
    Vec4 rB = Rotate(b.Orientation, j.LocalAnchorB);
    Vec4 d  = (b.Position + rB) - (a.Position + rA);
    const float len = Length3(d);
    if (len < 1e-6f) return;
    Vec4 dir = d / len;
    const float C = std::clamp(len - j.Length, -kMaxCorrection, kMaxCorrection);

    const Mat4 invIa = FreshInvInertia(a);
    const Mat4 invIb = FreshInvInertia(b);
    Vec4 raxd = Cross(rA, dir);
    Vec4 rbxd = Cross(rB, dir);
    const float k = a.InvMass + b.InvMass
                  + Dot3(raxd, Transform(raxd, invIa))
                  + Dot3(rbxd, Transform(rbxd, invIb));
    if (k <= 0.0f) return;
    ApplyPositional(a, b, rA, rB, dir * (-C / k), invIa, invIb);
}

// ============================== BallJoint ==================================

void Presolve(std::vector<RigidBody>& bodies, BallJoint& j) {
    RigidBody& a = bodies[j.A];
    RigidBody& b = bodies[j.B];

    j.rA   = Rotate(a.Orientation, j.LocalAnchorA);
    j.rB   = Rotate(b.Orientation, j.LocalAnchorB);
    j.InvK = Inverse3x3(BallK(a, b, j.rA, j.rB, a.InvInertiaWorld, b.InvInertiaWorld));
}

void WarmStart(std::vector<RigidBody>& bodies, BallJoint& j) {
    ApplyImpulse(bodies[j.A], bodies[j.B], j.rA, j.rB, j.AccumImpulse);
}

void Solve(std::vector<RigidBody>& bodies, BallJoint& j) {
    RigidBody& a = bodies[j.A];
    RigidBody& b = bodies[j.B];

    Vec4 Cdot   = VelocityAt(b, j.rB) - VelocityAt(a, j.rA);
    Vec4 lambda = Transform(-Cdot, j.InvK);         // 3x3 block solve
    j.AccumImpulse += lambda;
    ApplyImpulse(a, b, j.rA, j.rB, lambda);
}

void SolvePosition(std::vector<RigidBody>& bodies, BallJoint& j) {
    RigidBody& a = bodies[j.A];
    RigidBody& b = bodies[j.B];

    const Mat4 invIa = FreshInvInertia(a);
    const Mat4 invIb = FreshInvInertia(b);
    Vec4 rA = Rotate(a.Orientation, j.LocalAnchorA);
    Vec4 rB = Rotate(b.Orientation, j.LocalAnchorB);
    Vec4 C  = ClampLength3((b.Position + rB) - (a.Position + rA), kMaxCorrection);
    Vec4 lambda = Transform(-C, Inverse3x3(BallK(a, b, rA, rB, invIa, invIb)));
    ApplyPositional(a, b, rA, rB, lambda, invIa, invIb);
}

// ============================== HingeJoint =================================

void Presolve(std::vector<RigidBody>& bodies, HingeJoint& j) {
    RigidBody& a = bodies[j.A];
    RigidBody& b = bodies[j.B];

    // Ball block, identical to BallJoint.
    j.rA   = Rotate(a.Orientation, j.LocalAnchorA);
    j.rB   = Rotate(b.Orientation, j.LocalAnchorB);
    j.InvK = Inverse3x3(BallK(a, b, j.rA, j.rB, a.InvInertiaWorld, b.InvInertiaWorld));

    // Two angular rows: keep A's axis perpendicular to B's frozen b2/c2.
    // For C1 = axis . b2:  Cdot1 = (axis x b2) . (wA - wB)  -> row axis U.
    Vec4 axis = Rotate(a.Orientation, j.LocalAxisA);
    Vec4 perp[2] = { Rotate(b.Orientation, j.LocalB2),
                     Rotate(b.Orientation, j.LocalC2) };
    for (int t = 0; t < 2; ++t) {
        j.U[t] = Cross(axis, perp[t]);
        const float k = Dot3(j.U[t], Transform(j.U[t], a.InvInertiaWorld))
                      + Dot3(j.U[t], Transform(j.U[t], b.InvInertiaWorld));
        j.AxialMass[t] = k > 1e-9f ? 1.0f / k : 0.0f;
    }
}

void WarmStart(std::vector<RigidBody>& bodies, HingeJoint& j) {
    RigidBody& a = bodies[j.A];
    RigidBody& b = bodies[j.B];
    ApplyImpulse(a, b, j.rA, j.rB, j.AccumImpulse);
    for (int t = 0; t < 2; ++t)
        ApplyAngular(a, b, j.U[t], j.AccumAxial[t]);
}

void Solve(std::vector<RigidBody>& bodies, HingeJoint& j) {
    RigidBody& a = bodies[j.A];
    RigidBody& b = bodies[j.B];

    // Ball block.
    Vec4 Cdot   = VelocityAt(b, j.rB) - VelocityAt(a, j.rA);
    Vec4 lambda = Transform(-Cdot, j.InvK);
    j.AccumImpulse += lambda;
    ApplyImpulse(a, b, j.rA, j.rB, lambda);

    // Axis-alignment rows.
    for (int t = 0; t < 2; ++t) {
        const float cdot = Dot3(j.U[t], a.AngularVelocity - b.AngularVelocity);
        const float L    = -j.AxialMass[t] * cdot;
        j.AccumAxial[t] += L;
        ApplyAngular(a, b, j.U[t], L);
    }
}

void SolvePosition(std::vector<RigidBody>& bodies, HingeJoint& j) {
    RigidBody& a = bodies[j.A];
    RigidBody& b = bodies[j.B];

    // Angular error first (the ball projection below preserves it).
    {
        const Mat4 invIa = FreshInvInertia(a);
        const Mat4 invIb = FreshInvInertia(b);
        Vec4 axis = Rotate(a.Orientation, j.LocalAxisA);
        Vec4 perp[2] = { Rotate(b.Orientation, j.LocalB2),
                         Rotate(b.Orientation, j.LocalC2) };
        for (int t = 0; t < 2; ++t) {
            Vec4 u = Cross(axis, perp[t]);
            const float k = Dot3(u, Transform(u, invIa))
                          + Dot3(u, Transform(u, invIb));
            if (k <= 1e-9f) continue;
            const float C = std::clamp(Dot3(axis, perp[t]), -kMaxCorrection, kMaxCorrection);
            const float L = -C / k;
            RotateBy(a, Transform(u * L, invIa));
            RotateBy(b, -Transform(u * L, invIb));
        }
    }

    // Ball anchor error (fresh inertia again — the rotations just above moved
    // both bodies' frames).
    const Mat4 invIa = FreshInvInertia(a);
    const Mat4 invIb = FreshInvInertia(b);
    Vec4 rA = Rotate(a.Orientation, j.LocalAnchorA);
    Vec4 rB = Rotate(b.Orientation, j.LocalAnchorB);
    Vec4 C  = ClampLength3((b.Position + rB) - (a.Position + rA), kMaxCorrection);
    Vec4 lambda = Transform(-C, Inverse3x3(BallK(a, b, rA, rB, invIa, invIb)));
    ApplyPositional(a, b, rA, rB, lambda, invIa, invIb);
}

} // namespace SGE::Physics::Joints
