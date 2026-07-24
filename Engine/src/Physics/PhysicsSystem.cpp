#include "Physics/PhysicsSystem.h"
#include "Physics/PhysicsComponents.h"
#include "Physics/PhysicsWorld.h"
#include "Scene/World.h"
#include "Scene/Components.h"

namespace SGE {

void PhysicsSystem::SyncTransforms(World& world, const Physics::PhysicsWorld& physics) {
    // RigidBodyComponent drives the view (rarer than TransformComponent).
    world.View<RigidBodyComponent, TransformComponent>(
        [&](Entity, RigidBodyComponent& rb, TransformComponent& tc) {
            const Physics::RigidBody& body = physics.Body(rb.Body);

            float p[4]; body.Position.Store(p);
            tc.Position = { p[0], p[1], p[2] };

            alignas(16) float q[4];
            _mm_store_ps(q, body.Orientation.v);
            tc.RotationQuat  = { q[0], q[1], q[2], q[3] };
            tc.UseQuaternion = true;
        });
}

} // namespace SGE
