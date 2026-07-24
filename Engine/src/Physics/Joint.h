#pragma once
// ---------------------------------------------------------------------------
// Joints — velocity constraints solved by the same sequential-impulse
// machinery as contacts (presolve / warm-start / iterate), run BEFORE the
// contact rows inside each solver iteration.
//
//   DistanceJoint  1 row   rigid rod between two anchor points (both signs,
//                          no clamp). C = |pB - pA| - restLength.
//   BallJoint      3 rows  pins two anchor points together. Solved as one
//                          3x3 block: K = (1/mA + 1/mB) I3
//                                       - skew(rA) Ia^-1 skew(rA)
//                                       - skew(rB) Ib^-1 skew(rB),
//                          lambda = K^-1 * -(Cdot + bias) via Math::Inverse3x3
//                          (block solve converges far faster than 3 scalar
//                          rows for chains).
//   HingeJoint     3+2     ball block plus two angular rows that keep A's
//                          hinge axis perpendicular to a b2/c2 basis frozen
//                          in B at creation (axis x b2 and axis x c2 rows).
//
// Position drift is NOT fed into the velocity rows (no Baumgarte bias):
// a bias velocity does real work on the body, and on a fast-spinning joint the
// (omega*h)^2 anchor drift grows with speed — the correction then pumps energy
// in a feedback loop until the joint slingshots. Instead SolvePosition runs
// after integration as a nonlinear Gauss-Seidel pass that PROJECTS the error
// out by moving positions/orientations directly, changing no velocities and
// therefore no kinetic energy (the Box2D scheme).
// All joints warm start their accumulated impulses.
// ---------------------------------------------------------------------------
#include "RigidBody.h"
#include <vector>

namespace SGE::Physics {

struct alignas(16) DistanceJoint {
    Math::Vec4 LocalAnchorA, LocalAnchorB;  // body-space attach points
    BodyHandle A = 0, B = 0;
    float      Length       = 1.0f;
    float      AccumImpulse = 0.0f;

    // presolve caches
    Math::Vec4 rA, rB, Dir;
    float Mass = 0.0f;
};

struct alignas(16) BallJoint {
    Math::Vec4 LocalAnchorA, LocalAnchorB;
    BodyHandle A = 0, B = 0;
    Math::Vec4 AccumImpulse;                // 3D impulse (w = 0)

    // presolve caches
    Math::Vec4 rA, rB;
    Math::Mat4 InvK;
};

struct alignas(16) HingeJoint {
    Math::Vec4 LocalAnchorA, LocalAnchorB;
    Math::Vec4 LocalAxisA;                  // hinge axis in A's frame
    Math::Vec4 LocalB2, LocalC2;            // basis perpendicular to the axis, in B's frame
    BodyHandle A = 0, B = 0;
    Math::Vec4 AccumImpulse;                // ball block
    float      AccumAxial[2] = { 0.0f, 0.0f };

    // presolve caches
    Math::Vec4 rA, rB;
    Math::Mat4 InvK;
    Math::Vec4 U[2];                        // angular row axes (axis x b2, axis x c2)
    float AxialMass[2] = { 0.0f, 0.0f };
};

namespace Joints {

void Presolve(std::vector<RigidBody>& bodies, DistanceJoint& j);
void Presolve(std::vector<RigidBody>& bodies, BallJoint& j);
void Presolve(std::vector<RigidBody>& bodies, HingeJoint& j);

void WarmStart(std::vector<RigidBody>& bodies, DistanceJoint& j);
void WarmStart(std::vector<RigidBody>& bodies, BallJoint& j);
void WarmStart(std::vector<RigidBody>& bodies, HingeJoint& j);

void Solve(std::vector<RigidBody>& bodies, DistanceJoint& j);
void Solve(std::vector<RigidBody>& bodies, BallJoint& j);
void Solve(std::vector<RigidBody>& bodies, HingeJoint& j);

// Post-integration position projection (NGS). Moves positions/orientations to
// close the constraint error directly; velocities are untouched.
void SolvePosition(std::vector<RigidBody>& bodies, DistanceJoint& j);
void SolvePosition(std::vector<RigidBody>& bodies, BallJoint& j);
void SolvePosition(std::vector<RigidBody>& bodies, HingeJoint& j);

} // namespace Joints
} // namespace SGE::Physics
