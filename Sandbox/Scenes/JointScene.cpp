#include "Scenes/JointScene.h"

#include "Scene/Components.h"
#include "Scene/RenderSystem.h"
#include "Physics/PhysicsComponents.h"
#include "Physics/PhysicsSystem.h"

#include "imgui.h"

using namespace SGE;
using Math::Quat;
using Math::Vec4;

const char* JointScene::Description() const {
    return "Joints as velocity constraints in the same sequential-impulse loop "
           "as contacts: ball joints (3x3 block solve), a hinge (ball block + "
           "two axis rows) and distance rods. Positional drift is projected "
           "out after integration (NGS), so nothing gains energy from the "
           "correction. Pick a rig, poke it, and switch the solver between "
           "single-threaded and JobSystem execution.";
}

void JointScene::OnLoad(const DemoContext&) {
    m_jobs.Initialize();
    BuildRig();
}

void JointScene::OnUnload() {
    m_jobs.Shutdown();
    m_world = World{};
    m_physics.Clear();
}

Physics::BodyHandle JointScene::SpawnSphere(DirectX::XMFLOAT3 pos, float radius, float mass,
                                            DirectX::XMFLOAT4 color) {
    Physics::BodyHandle h = m_physics.AddSphere(Vec4(pos.x, pos.y, pos.z, 0), radius, mass);
    Entity e = m_world.Create();
    TransformComponent tc;
    tc.Position = pos;
    tc.Scale    = { radius, radius, radius };
    m_world.Add(e, tc);
    MeshComponent mc; mc.MeshPtr = m_sphere;
    m_world.Add(e, mc);
    MaterialComponent mat; mat.BaseColor = color;
    m_world.Add(e, mat);
    m_world.Add(e, RigidBodyComponent{ h });
    return h;
}

Physics::BodyHandle JointScene::SpawnBox(DirectX::XMFLOAT3 pos, DirectX::XMFLOAT3 he,
                                         float mass, DirectX::XMFLOAT4 color) {
    Physics::BodyHandle h = m_physics.AddBox(Vec4(pos.x, pos.y, pos.z, 0), Quat(),
                                             Vec4(he.x, he.y, he.z, 0), mass);
    Entity e = m_world.Create();
    TransformComponent tc;
    tc.Position = pos;
    tc.Scale    = { he.x * 2.0f, he.y * 2.0f, he.z * 2.0f };
    m_world.Add(e, tc);
    MeshComponent mc; mc.MeshPtr = m_cube;
    m_world.Add(e, mc);
    MaterialComponent mat; mat.BaseColor = color;
    m_world.Add(e, mat);
    m_world.Add(e, RigidBodyComponent{ h });
    return h;
}

void JointScene::BuildRig() {
    m_world = World{};
    m_physics.Clear();
    m_hasPoke = false;

    // Shared floor (visual slab + infinite collision plane).
    m_physics.AddStaticPlane(Vec4(0, 1, 0, 0), 0.0f);
    Entity ground = m_world.Create();
    TransformComponent tc;
    tc.Position = { 0.0f, -0.1f, 0.0f };
    tc.Scale    = { 30.0f, 0.2f, 30.0f };
    m_world.Add(ground, tc);
    MeshComponent mc; mc.MeshPtr = m_cube;
    m_world.Add(ground, mc);
    MaterialComponent mat; mat.BaseColor = { 0.35f, 0.37f, 0.40f, 1.0f };
    m_world.Add(ground, mat);

    const DirectX::XMFLOAT4 kAnchor{ 0.9f, 0.35f, 0.2f, 1.0f };
    const DirectX::XMFLOAT4 kLink  { 0.8f, 0.8f,  0.85f, 1.0f };

    switch (m_rig) {
    case 0: { // Hanging chain of ball-jointed links.
        const float top = 9.0f, spacing = 0.62f;
        Physics::BodyHandle prev = SpawnSphere({ 0, top, 0 }, 0.15f, 0.0f, kAnchor);
        for (int i = 1; i <= m_linkCount; ++i) {
            Physics::BodyHandle link =
                SpawnBox({ 0, top - spacing * float(i), 0 }, { 0.08f, 0.26f, 0.08f }, 1.0f, kLink);
            m_physics.AddBallJoint(prev, link, Vec4(0, top - spacing * (float(i) - 0.5f), 0, 0));
            prev = link;
        }
        m_pokeTarget = prev;
        m_hasPoke    = true;
        break;
    }
    case 1: { // Trapdoor on a hinge, with a ball to drop through it.
        Physics::BodyHandle frame =
            SpawnBox({ 0, 4.0f, -1.4f }, { 1.2f, 0.1f, 0.1f }, 0.0f, kAnchor);
        Physics::BodyHandle door =
            SpawnBox({ 0, 4.0f, 0.0f }, { 1.1f, 0.05f, 1.25f }, 2.0f, kLink);
        m_physics.AddHingeJoint(frame, door, Vec4(0, 4.0f, -1.28f, 0), Vec4(1, 0, 0, 0));
        m_pokeTarget = SpawnSphere({ 0.15f, 8.0f, 0.2f }, 0.45f, 2.0f, { 0.4f, 0.7f, 1.0f, 1.0f });
        m_hasPoke    = false;
        break;
    }
    case 2: { // Wrecking ball on a distance-joint rope vs a box stack.
        for (int i = 0; i < 5; ++i)
            SpawnBox({ -3.0f, 0.5f + float(i), 0 }, { 0.5f, 0.5f, 0.5f }, 1.0f, kLink);

        const float top = 8.0f, seg = 0.9f;
        Physics::BodyHandle prev = SpawnSphere({ 0, top, 0 }, 0.15f, 0.0f, kAnchor);
        DirectX::XMFLOAT3 p{ 0, top, 0 };
        for (int i = 1; i <= 4; ++i) {
            DirectX::XMFLOAT3 np{ seg * float(i), top, 0 };   // held horizontal, then released
            Physics::BodyHandle link = SpawnSphere(np, 0.12f, 0.4f, kLink);
            m_physics.AddDistanceJoint(prev, link, Vec4(p.x, p.y, p.z, 0), Vec4(np.x, np.y, np.z, 0));
            prev = link;
            p = np;
        }
        DirectX::XMFLOAT3 bp{ seg * 5.0f, top, 0 };
        Physics::BodyHandle ball = SpawnSphere(bp, 0.55f, 25.0f, { 0.25f, 0.25f, 0.28f, 1.0f });
        m_physics.AddDistanceJoint(prev, ball, Vec4(p.x, p.y, p.z, 0), Vec4(bp.x, bp.y, bp.z, 0));
        m_pokeTarget = ball;
        m_hasPoke    = true;
        break;
    }
    }
}

void JointScene::OnUpdate(const DemoContext& ctx) {
    m_physics.Update(ctx.dt, m_multithreaded ? &m_jobs : nullptr);
    PhysicsSystem::SyncTransforms(m_world, m_physics);
}

void JointScene::OnRender(const DemoContext& ctx) {
    ctx.renderSystem->Render(m_world, *ctx.camera, *ctx.objectCB, ctx.cmd, ctx.rootParamIndexCBV);
}

void JointScene::OnImGui() {
    ImGui::TextWrapped(
        "Every rig is held together by the engine's own joint rows — solved "
        "with the contacts each substep, warm-started, drift-corrected by "
        "position projection.");
    ImGui::Separator();

    const char* rigs[] = { "Hanging chain (ball joints)",
                           "Trapdoor (hinge)",
                           "Wrecking ball (distance rods)" };
    if (ImGui::Combo("Rig", &m_rig, rigs, 3))
        BuildRig();
    if (m_rig == 0 && ImGui::SliderInt("Chain links", &m_linkCount, 2, 14))
        BuildRig();

    if (m_hasPoke) {
        if (ImGui::Button("Poke")) {
            auto& b = m_physics.Body(m_pokeTarget);
            b.LinearVelocity += Vec4(-4.0f, 0.5f, 1.5f, 0) * (m_rig == 2 ? 0.4f : 1.0f);
        }
        ImGui::SameLine();
    }
    if (ImGui::Button("Rebuild rig"))
        BuildRig();

    ImGui::Separator();
    ImGui::SliderInt("Solver iterations", &m_physics.Iterations, 1, 30);
    ImGui::Checkbox("Multithreaded step", &m_multithreaded);

    const auto& t = m_physics.Timings();
    ImGui::Text("Bodies %u | joints %u | contacts %u",
                m_physics.BodyCount(), m_physics.JointCount(), t.Contacts);
    ImGui::Text("step %.3f ms (int %.2f broad %.2f narrow %.2f solve %.2f)",
                t.TotalMs, t.IntegrateMs, t.BroadMs, t.NarrowMs, t.SolveMs);
}
