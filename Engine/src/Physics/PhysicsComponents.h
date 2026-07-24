#pragma once
// ---------------------------------------------------------------------------
// ECS-side view of a physics body. The component is JUST a handle — bodies
// live in PhysicsWorld's dense array (where the solver and JobSystem want
// them), entities only reference one. PhysicsSystem::SyncTransforms copies
// the simulated pose back into TransformComponent after each update.
// ---------------------------------------------------------------------------
#include "Physics/RigidBody.h"

namespace SGE {

struct RigidBodyComponent {
    Physics::BodyHandle Body = 0;
};

} // namespace SGE
