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
// (Sphere < Box < Plane); narrowphase then only needs one function per
// unordered pair. Planes are always static and always canonical-B.
enum class ShapeType : uint32_t { Sphere = 0, Box = 1, Plane = 2 };

struct alignas(16) Collider {
    // Box: half-extents in xyz (w unused).
    // Plane: xyz = unit normal, w = d in the plane equation n.p = d.
    Math::Vec4 Extents;
    float      Radius = 0.5f;               // Sphere only
    ShapeType  Type   = ShapeType::Sphere;
};

struct alignas(16) RigidBody {
    Math::Vec4 Position;
    Math::Quat Orientation;
    Math::Vec4 LinearVelocity;
    Math::Vec4 AngularVelocity;             // world-space, rad/s
    Math::Vec4 InvInertiaBodyDiag;          // diagonal of body-space I^-1 (zero for static)
    Math::Mat4 InvInertiaWorld;             // refreshed at the top of every substep
    float      InvMass     = 0.0f;          // 0 = static
    float      Restitution = 0.2f;
    float      Friction    = 0.5f;
    Collider   Shape;

    bool IsStatic() const { return InvMass == 0.0f; }
};

} // namespace SGE::Physics
