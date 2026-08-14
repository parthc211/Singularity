#pragma once
// ---------------------------------------------------------------------------
// Rigid-body data — the flat POD the whole physics module operates on.
//
// Bodies live in one dense, 16-byte-aligned array inside PhysicsWorld and are
// addressed by BodyHandle (a plain index; v1 is append-only / bulk-Clear, so
// handles never go stale). Everything is SGE::Math types — the hand-written
// SIMD library is the only math the solver touches; DirectXMath appears solely
// at the ECS/render boundary.
//
// Conventions (identical to SimdMath.h): row-major matrices, row-vector
// transforms, quats (x,y,z,w). World-space inverse inertia is
//   I_w^-1 = R^T * diag(I_b^-1) * R      (R = ToMatrix(Orientation))
// and is APPLIED as  dOmega = Transform(angularImpulse, InvInertiaWorld) —
// this pairing is validated as a MathBenchmark correctness row before the
// solver ever uses it.
// ---------------------------------------------------------------------------
#include "Math/SimdMath.h"
#include <cstdint>

namespace SGE::Physics {

// Index into PhysicsWorld's body array. Stable for the life of the world
// (bodies are never removed individually, only bulk-cleared).
using BodyHandle = uint32_t;

// Ordered so mixed collision pairs can be canonicalized by type value
// (Sphere < Capsule < Box < Plane); narrowphase then only needs one function
// per unordered pair. Planes are always static and always canonical-B.
enum class ShapeType : uint32_t { Sphere = 0, Capsule = 1, Box = 2, Plane = 3 };

struct alignas(16) Collider {
    // Box:     half-extents in xyz (w unused).
    // Capsule: Extents.x = segment HALF-LENGTH (caps not included); the axis
    //          is local +Y; Radius = cap/side radius.
    // Plane:   xyz = unit normal, w = d in the plane equation n.p = d.
    Math::Vec4 Extents;
    float      Radius = 0.5f;               // Sphere / Capsule
    ShapeType  Type   = ShapeType::Sphere;
};

struct alignas(16) RigidBody {
    Math::Vec4 Position;
    Math::Quat Orientation;
    // Pose at the START of the current substep — rendering interpolates
    // between this and the current pose by the accumulator fraction, so the
    // fixed 120 Hz sim doesn't judder against an arbitrary frame rate.
    Math::Vec4 PrevPosition;
    Math::Quat PrevOrientation;
    Math::Vec4 LinearVelocity;
    Math::Vec4 AngularVelocity;             // world-space, rad/s
    // Split-impulse pseudo velocities: accumulated by the contact position
    // pass, consumed as pure displacement at integration, then zeroed — they
    // never mix into the real velocities, so penetration correction cannot
    // inject kinetic energy (no "Baumgarte bounce").
    Math::Vec4 PseudoLinearVelocity;
    Math::Vec4 PseudoAngularVelocity;
    Math::Vec4 InvInertiaBodyDiag;          // diagonal of body-space I^-1 (zero for static)
    Math::Mat4 InvInertiaWorld;             // refreshed at the top of every substep
    float      InvMass     = 0.0f;          // 0 = static
    float      Restitution = 0.2f;
    float      Friction    = 0.5f;
    Collider   Shape;

    // Sleeping (see PhysicsWorld's island logic): a body whose whole contact/
    // joint island has been quiet for TimeToSleep is frozen — integration and
    // its pair collection are skipped until something wakes the island.
    //
    // "Quiet" is measured by NET DISPLACEMENT from an anchor pose, not by
    // instantaneous velocity: warm-started stacks jitter with zero-mean
    // velocities forever, but they don't go anywhere — the anchor sees that.
    Math::Vec4 SleepAnchorPos;              // pose when the quiet window began
    Math::Quat SleepAnchorOrient;
    float      SleepTimer  = 0.0f;          // seconds inside the anchor tolerance
    bool       Sleeping    = false;

    bool IsStatic()   const { return InvMass == 0.0f; }
    // Doesn't move this step — used to skip pairs where nothing can change.
    bool IsImmobile() const { return IsStatic() || Sleeping; }
};

} // namespace SGE::Physics
