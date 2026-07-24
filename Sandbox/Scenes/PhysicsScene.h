#pragma once
#include "Scene/DemoScene.h"
#include "Scene/World.h"
#include "Physics/PhysicsWorld.h"
#include "Jobs/JobSystem.h"

#include <DirectXMath.h>
#include <cstdint>
#include <random>

namespace SGE { class Mesh; }

// ---------------------------------------------------------------------------
// Rigid-Body Physics — hand-written impulse solver (Phase 11).
//
// Everything simulated here comes from Engine/src/Physics: sphere/box/plane
// colliders, a sequential-impulse contact solver with friction, restitution
// and warm starting, and a spatial-hash broadphase — all built on the
// hand-written SGE::Math SIMD library (no PhysX/Bullet/Jolt, and DirectXMath
// only at the render boundary).
//
// The panel spawns stacks and body rain, exposes the solver's tunables live
// (gravity, friction, restitution, iterations), toggles a contact-point
// overlay (the tool the box-box SAT was debugged with), and A/B-switches the
// step between single-threaded and JobSystem-parallel execution — the
// trajectory is bit-identical either way, only the milliseconds change.
// ---------------------------------------------------------------------------
class PhysicsScene : public SGE::DemoScene
{
public:
    PhysicsScene(SGE::Mesh* cube, SGE::Mesh* sphere) : m_cube(cube), m_sphere(sphere) {}

    const char* Name()        const override { return "Physics: Stacks & Rain"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override
    {
        pos[0] = 0.0f; pos[1] = 9.0f; pos[2] = -22.0f;
        yaw = 0.0f; pitch = -0.25f;
        return true;
    }

private:
    void BuildScene();
    void SpawnSphere(DirectX::XMFLOAT3 pos, float radius, float mass);
    void SpawnBox(DirectX::XMFLOAT3 pos, DirectX::XMFLOAT3 halfExtents, float mass,
                  const SGE::Math::Quat& orientation = SGE::Math::Quat());
    void SpawnStack(int count);
    void SpawnRain(int count);
    void DrawContactOverlay() const;

    SGE::Mesh* m_cube   = nullptr;
    SGE::Mesh* m_sphere = nullptr;

    SGE::World                 m_world;
    SGE::Physics::PhysicsWorld m_physics;
    SGE::JobSystem             m_jobs;
    std::mt19937               m_rng{ 1234u };

    // Spawn-time material defaults (live bodies keep what they were born with).
    float m_restitution = 0.2f;
    float m_friction    = 0.5f;
    int   m_stackCount  = 10;

    bool  m_multithreaded = true;
    bool  m_overlay       = false;
    float m_msSingle      = 0.0f;   // rolling averages for the A/B readout
    float m_msMulti       = 0.0f;

    // Captured per-frame for the ImGui-space contact overlay.
    DirectX::XMFLOAT4X4 m_viewProj = {};
    uint32_t            m_width = 1, m_height = 1;
};
