#pragma once
// ---------------------------------------------------------------------------
// Narrowphase — exact contact generation per shape pair.
//
// Every routine takes bodies in CANONICAL order (a.Shape.Type <= b.Shape.Type,
// ties broken by lower body index first — PhysicsWorld arranges this) and
// fills the manifold's Normal (unit, pointing from a toward b), Points
// (Position / Penetration / FeatureId) and Count. A/B handles are set by the
// caller. Returns true if the shapes touch.
//
// Pure functions of the two bodies — no globals, no allocation — so the pair
// loop can run them from JobSystem workers into per-pair output slots.
// ---------------------------------------------------------------------------
#include "Contact.h"

namespace SGE::Physics::Narrowphase {

bool SphereSphere(const RigidBody& a, const RigidBody& b, Manifold& m);
bool SpherePlane (const RigidBody& sphere, const RigidBody& plane, Manifold& m);
bool SphereBox   (const RigidBody& sphere, const RigidBody& box,   Manifold& m);
bool BoxPlane    (const RigidBody& box,    const RigidBody& plane, Manifold& m);
bool BoxBox      (const RigidBody& a, const RigidBody& b, Manifold& m);

// Dispatch on the (already canonical) shape pair.
bool Collide(const RigidBody& a, const RigidBody& b, Manifold& m);

} // namespace SGE::Physics::Narrowphase
