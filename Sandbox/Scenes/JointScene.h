#pragma once
#include "Scene/DemoScene.h"
#include "Scene/World.h"
#include "Physics/PhysicsWorld.h"
#include "Jobs/JobSystem.h"

#include <DirectXMath.h>

namespace SGE { class Mesh; }

// ---------------------------------------------------------------------------
// Joints — constraint solving on top of the rigid-body core (Phase 11).
//
// Three rigs, one per joint type the engine implements:
//   Hanging chain   BallJoints   — 3x3 block solve per link (Math::Inverse3x3)
//   Trapdoor        HingeJoint   — ball block + two axis-alignment rows
//   Wrecking ball   DistanceJoints — scalar rod rows swinging a heavy sphere
//                                    into a box stack (contacts + joints
//                                    solved together)
//
// Joints run inside the same sequential-impulse loop as contacts, warm-started,
// with positional drift removed by an energy-neutral NGS projection after
// integration (no Baumgarte slingshot on fast swings).
// ---------------------------------------------------------------------------
class JointScene : public SGE::DemoScene
{
public:
    JointScene(SGE::Mesh* cube, SGE::Mesh* sphere) : m_cube(cube), m_sphere(sphere) {}

    const char* Name()        const override { return "Physics: Joints"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override
    {
        pos[0] = 0.0f; pos[1] = 6.0f; pos[2] = -16.0f;
        yaw = 0.0f; pitch = -0.12f;
        return true;
    }

private:
    void BuildRig();
    SGE::Physics::BodyHandle SpawnSphere(DirectX::XMFLOAT3 pos, float radius, float mass,
                                         DirectX::XMFLOAT4 color);
    SGE::Physics::BodyHandle SpawnBox(DirectX::XMFLOAT3 pos, DirectX::XMFLOAT3 halfExtents,
                                      float mass, DirectX::XMFLOAT4 color);

    SGE::Mesh* m_cube   = nullptr;
    SGE::Mesh* m_sphere = nullptr;

    SGE::World                 m_world;
    SGE::Physics::PhysicsWorld m_physics;
    SGE::JobSystem             m_jobs;

    int  m_rig           = 0;        // 0 chain, 1 trapdoor, 2 wrecking ball
    int  m_linkCount     = 8;
    bool m_multithreaded = true;

    SGE::Physics::BodyHandle m_pokeTarget = 0;  // chain end / wrecking ball
    bool                     m_hasPoke    = false;
};
