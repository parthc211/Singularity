#include "Scenes/PhysicsScene.h"

#include "Scene/Components.h"
#include "Scene/RenderSystem.h"
#include "Physics/PhysicsComponents.h"
#include "Physics/PhysicsSystem.h"
#include "Core/Camera.h"
#include "Renderer/Renderer.h"

#include "imgui.h"

#include <algorithm>

using namespace SGE;
using Math::Quat;
using Math::Vec4;

const char* PhysicsScene::Description() const {
    return "Hand-written 3D rigid-body physics: sphere/box/plane colliders, a "
           "sequential-impulse solver (accumulated + clamped impulses, warm "
           "starting, Baumgarte bias) at 120 Hz fixed substeps, and a spatial-"
           "hash broadphase. Built entirely on the engine's own SIMD math. The "
           "step runs single-threaded or on the work-stealing JobSystem — same "
           "trajectory, different milliseconds.";
}

void PhysicsScene::OnLoad(const DemoContext&) {
    m_jobs.Initialize();            // (cores - 1) workers, like JobScene
    BuildScene();
}

void PhysicsScene::OnUnload() {
    m_jobs.Shutdown();
    m_world = World{};
    m_physics.Clear();
}

void PhysicsScene::BuildScene() {
    m_world = World{};
    m_physics.Clear();
    m_rng.seed(1234u);

    // Physics ground: an infinite static plane. Visual ground: a thin slab
    // whose top face sits exactly on it (no RigidBodyComponent — the plane is
    // the collider, the slab is scenery).
    m_physics.AddStaticPlane(Vec4(0, 1, 0, 0), 0.0f);
    Entity ground = m_world.Create();
    TransformComponent tc;
    tc.Position = { 0.0f, -0.1f, 0.0f };
    tc.Scale    = { 40.0f, 0.2f, 40.0f };
    m_world.Add(ground, tc);
    MeshComponent mc; mc.MeshPtr = m_cube;
    m_world.Add(ground, mc);
    MaterialComponent mat; mat.BaseColor = { 0.35f, 0.37f, 0.40f, 1.0f };
    m_world.Add(ground, mat);

    SpawnStack(m_stackCount);
}

void PhysicsScene::SpawnSphere(DirectX::XMFLOAT3 pos, float radius, float mass) {
    Physics::BodyHandle h = m_physics.AddSphere(Vec4(pos.x, pos.y, pos.z, 0), radius, mass);
    m_physics.Body(h).Restitution = m_restitution;
    m_physics.Body(h).Friction    = m_friction;

    std::uniform_real_distribution<float> hue(0.35f, 1.0f);
    Entity e = m_world.Create();
    TransformComponent tc;
    tc.Position = pos;
    tc.Scale    = { radius, radius, radius };        // unit-radius mesh
    m_world.Add(e, tc);
    MeshComponent mc; mc.MeshPtr = m_sphere;
    m_world.Add(e, mc);
    MaterialComponent mat; mat.BaseColor = { hue(m_rng), hue(m_rng), hue(m_rng), 1.0f };
    m_world.Add(e, mat);
    m_world.Add(e, RigidBodyComponent{ h });
}

void PhysicsScene::SpawnBox(DirectX::XMFLOAT3 pos, DirectX::XMFLOAT3 he, float mass,
                            const Quat& orientation) {
    Physics::BodyHandle h = m_physics.AddBox(Vec4(pos.x, pos.y, pos.z, 0), orientation,
                                             Vec4(he.x, he.y, he.z, 0), mass);
    m_physics.Body(h).Restitution = m_restitution;
    m_physics.Body(h).Friction    = m_friction;

    std::uniform_real_distribution<float> hue(0.35f, 1.0f);
    Entity e = m_world.Create();
    TransformComponent tc;
    tc.Position = pos;
    tc.Scale    = { he.x * 2.0f, he.y * 2.0f, he.z * 2.0f }; // unit cube -> full extents
    m_world.Add(e, tc);
    MeshComponent mc; mc.MeshPtr = m_cube;
    m_world.Add(e, mc);
    MaterialComponent mat; mat.BaseColor = { hue(m_rng), hue(m_rng), hue(m_rng), 1.0f };
    m_world.Add(e, mat);
    m_world.Add(e, RigidBodyComponent{ h });
}

void PhysicsScene::SpawnStack(int count) {
    for (int i = 0; i < count; ++i)
        SpawnBox({ 0.0f, 0.5f + 1.0f * float(i), 0.0f }, { 0.5f, 0.5f, 0.5f }, 1.0f);
}

void PhysicsScene::SpawnRain(int count) {
    std::uniform_real_distribution<float> px(-6.0f, 6.0f), py(8.0f, 22.0f);
    std::uniform_real_distribution<float> sz(0.25f, 0.5f), ang(0.0f, 3.1f), axis(-1.0f, 1.0f);
    for (int i = 0; i < count; ++i) {
        DirectX::XMFLOAT3 p{ px(m_rng), py(m_rng), px(m_rng) };
        if (i & 1) {
            SpawnSphere(p, sz(m_rng), 1.0f);
        } else {
            Quat q = Quat::FromAxisAngle(Vec4(axis(m_rng), axis(m_rng), axis(m_rng), 0), ang(m_rng));
            float s = sz(m_rng);
            SpawnBox(p, { s, sz(m_rng), s }, 1.0f, q);
        }
    }
}

void PhysicsScene::OnUpdate(const DemoContext& ctx) {
    m_physics.Update(ctx.dt, m_multithreaded ? &m_jobs : nullptr);
    PhysicsSystem::SyncTransforms(m_world, m_physics);

    // Rolling per-mode average of the physics step cost (JobScene pattern).
    // High-FPS frames that owed no substep would drag the average toward zero.
    if (m_physics.Timings().Substeps > 0) {
        const float ms = m_physics.Timings().TotalMs;
        float& ema = m_multithreaded ? m_msMulti : m_msSingle;
        ema = ema == 0.0f ? ms : ema * 0.95f + ms * 0.05f;
    }

    // Capture what the ImGui-space overlay needs (OnImGui gets no context).
    DirectX::XMStoreFloat4x4(&m_viewProj, ctx.camera->GetViewProjection());
    m_width  = ctx.renderer->GetWidth();
    m_height = ctx.renderer->GetHeight();
}

void PhysicsScene::OnRender(const DemoContext& ctx) {
    ctx.renderSystem->Render(m_world, *ctx.camera, *ctx.objectCB, ctx.cmd, ctx.rootParamIndexCBV);
}

void PhysicsScene::DrawContactOverlay() const {
    using namespace DirectX;
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const XMMATRIX vp = XMLoadFloat4x4(&m_viewProj);

    // World -> pixel. Returns false when behind the camera.
    auto project = [&](Vec4 wpos, ImVec2& out) {
        float f[4]; wpos.Store(f);
        XMVECTOR clip = XMVector4Transform(XMVectorSet(f[0], f[1], f[2], 1.0f), vp);
        const float w = XMVectorGetW(clip);
        if (w < 0.01f) return false;
        out.x = (XMVectorGetX(clip) / w * 0.5f + 0.5f) * float(m_width);
        out.y = (0.5f - XMVectorGetY(clip) / w * 0.5f) * float(m_height);
        return true;
    };

    for (const auto& m : m_physics.Manifolds()) {
        for (uint32_t i = 0; i < m.Count; ++i) {
            ImVec2 p, q;
            if (!project(m.Points[i].Position, p)) continue;
            dl->AddCircleFilled(p, 3.5f, IM_COL32(255, 64, 64, 220));
            if (project(m.Points[i].Position + m.Normal * 0.35f, q))
                dl->AddLine(p, q, IM_COL32(255, 224, 64, 220), 2.0f);
        }
    }
}

void PhysicsScene::OnImGui() {
    ImGui::TextWrapped(
        "Every body below is simulated by the engine's own solver — no physics "
        "middleware. Warm-started sequential impulses let the stack rest; "
        "spawn rain to stress the spatial-hash broadphase.");
    ImGui::Separator();

    if (ImGui::Button("Drop sphere"))
        SpawnSphere({ 0.2f, 14.0f, 0.1f }, 0.5f, 1.0f);
    ImGui::SameLine();
    if (ImGui::Button("Drop box"))
        SpawnBox({ -0.2f, 14.0f, -0.1f }, { 0.4f, 0.4f, 0.4f }, 1.0f,
                 Quat::FromAxisAngle(Vec4(1, 1, 0, 0), 0.7f));
    ImGui::SameLine();
    if (ImGui::Button("Rain 50"))
        SpawnRain(50);

    ImGui::SliderInt("Stack height", &m_stackCount, 2, 12);
    if (ImGui::Button("Spawn stack"))
        SpawnStack(m_stackCount);
    ImGui::SameLine();
    if (ImGui::Button("Reset scene"))
        BuildScene();

    ImGui::Separator();
    float g[4]; m_physics.Gravity.Store(g);
    if (ImGui::SliderFloat("Gravity Y", &g[1], -25.0f, 0.0f, "%.2f"))
        m_physics.Gravity = Vec4(g[0], g[1], g[2], 0.0f);
    ImGui::SliderFloat("Restitution (new bodies)", &m_restitution, 0.0f, 0.95f, "%.2f");
    ImGui::SliderFloat("Friction (new bodies)", &m_friction, 0.0f, 1.0f, "%.2f");
    ImGui::SliderInt("Solver iterations", &m_physics.Iterations, 1, 30);

    ImGui::Separator();
    ImGui::Checkbox("Multithreaded step", &m_multithreaded);
    ImGui::SameLine();
    ImGui::Checkbox("Contact overlay", &m_overlay);
    ImGui::Checkbox("Spatial-hash broadphase", &m_physics.UseGrid);
    ImGui::SameLine();
    ImGui::TextDisabled("(off = brute force n^2)");

    const auto& t = m_physics.Timings();
    ImGui::Separator();
    ImGui::Text("Bodies %u | pairs %u | contacts %u | substeps %u",
                m_physics.BodyCount(), t.Pairs, t.Contacts, t.Substeps);
    ImGui::Text("integrate %.2f  broad %.2f  narrow %.2f  solve %.2f ms",
                t.IntegrateMs, t.BroadMs, t.NarrowMs, t.SolveMs);
    ImGui::Text("Step total (rolling avg):");
    ImGui::Text("  single-threaded: %.3f ms", m_msSingle);
    ImGui::Text("  multithreaded:   %.3f ms", m_msMulti);
    if (m_msSingle > 0.0f && m_msMulti > 0.0f)
        ImGui::Text("  speedup: %.2fx", m_msSingle / m_msMulti);
    ImGui::TextDisabled("(toggle the checkbox to populate both rows —");
    ImGui::TextDisabled(" the trajectory is identical in both modes)");

    if (m_overlay)
        DrawContactOverlay();
}
