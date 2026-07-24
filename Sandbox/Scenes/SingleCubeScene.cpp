#include "Scenes/SingleCubeScene.h"

#include "Scene/Components.h"
#include "Scene/RenderSystem.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"

#include "imgui.h"

using namespace SGE;

const char* SingleCubeScene::Description() const {
    return "One cube entity. Its spin, scale, and tint are pushed from the panel "
           "below into the entity's Transform/Material each frame — the same "
           "RenderSystem path as the grid, just a single object.";
}

void SingleCubeScene::OnLoad(const DemoContext&) {
    m_world  = World{};
    m_entity = m_world.Create();
    m_world.Add(m_entity, TransformComponent{});
    MeshComponent mc; mc.MeshPtr = m_cube;
    m_world.Add(m_entity, mc);
    m_world.Add(m_entity, MaterialComponent{});
}

void SingleCubeScene::OnUnload() { m_world = World{}; }

void SingleCubeScene::OnUpdate(const DemoContext& ctx) {
    auto& t = m_world.Get<TransformComponent>(m_entity);
    t.Rotation.y += ctx.dt * m_spin;
    t.Rotation.x += ctx.dt * m_spin * 0.5f;
    t.Scale = { m_scale, m_scale, m_scale };

    auto& mat = m_world.Get<MaterialComponent>(m_entity);
    mat.BaseColor = { m_color[0], m_color[1], m_color[2], 1.0f };
}

void SingleCubeScene::OnRender(const DemoContext& ctx) {
    ctx.renderSystem->Render(m_world, *ctx.camera, *ctx.objectCB, ctx.cmd, ctx.rootParamIndexCBV);
}

void SingleCubeScene::OnImGui() {
    ImGui::SliderFloat("Spin",  &m_spin,  0.0f, 4.0f, "%.2f");
    ImGui::SliderFloat("Scale", &m_scale, 0.25f, 3.0f, "%.2f");
    ImGui::ColorEdit3("Tint", m_color);
}
