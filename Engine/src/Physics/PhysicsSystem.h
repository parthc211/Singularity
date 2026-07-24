#pragma once
// ---------------------------------------------------------------------------
// The one-way bridge from simulation to rendering: after PhysicsWorld::Update,
// copy each body's position + orientation quaternion into the entity's
// TransformComponent (switching it to the quaternion rotation path). This is
// the ONLY place physics meets DirectXMath types — the solver itself never
// sees an XMFLOAT.
// ---------------------------------------------------------------------------

namespace SGE {

class World;
namespace Physics { class PhysicsWorld; }

class PhysicsSystem {
public:
    // For every entity with RigidBodyComponent + TransformComponent, write the
    // simulated pose into the transform (Scale is left alone — it is render
    // sugar; collider sizes live in the physics body).
    static void SyncTransforms(World& world, const Physics::PhysicsWorld& physics);
};

} // namespace SGE
