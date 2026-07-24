#include "Scenes/CubeGridScene.h"

#include "Scene/Components.h"
#include "Scene/RenderSystem.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"

#include "imgui.h"
#include <DirectXMath.h>
#include <cmath>

using namespace SGE;

const char* CubeGridScene::Description() const {
    return "An N x N grid of cubes. Each cube is an ECS entity carrying a "
           "Transform, a (shared) Mesh, and a Material tint. RenderSystem walks "
           "every Transform+Mesh entity and uploads its constants from a per-frame "
           "GPU arena, binding one root CBV per draw.";
}

void CubeGridScene::Rebuild() {
    m_world = World{}; // drop all entities and rebuild from scratch

    const int   n       = m_gridSize;
    const float spacing = 2.5f;
    const float offset  = (n - 1) * 0.5f * spacing; // center the grid on the origin

    int i = 0;
    for (int gx = 0; gx < n; ++gx) {
        for (int gz = 0; gz < n; ++gz, ++i) {
            Entity e = m_world.Create();

            TransformComponent t;
            t.Position = { gx * spacing - offset, 0.0f, gz * spacing - offset };
            m_world.Add(e, t);

            MeshComponent mc;
            mc.MeshPtr = m_cube;
            m_world.Add(e, mc);

            // Spread tints around the colour wheel so adjacent cubes differ.
            const float h = (n * n > 1) ? float(i) / float(n * n) : 0.0f;
            const float k = h * DirectX::XM_2PI;
            MaterialComponent mat;
            mat.BaseColor = {
                0.5f + 0.5f * sinf(k),
                0.5f + 0.5f * sinf(k + DirectX::XM_2PI / 3.0f),
                0.5f + 0.5f * sinf(k + 2.0f * DirectX::XM_2PI / 3.0f),
                1.0f
            };
            m_world.Add(e, mat);
        }
    }
}

void CubeGridScene::OnLoad(const DemoContext&) { Rebuild(); }
void CubeGridScene::OnUnload()                 { m_world = World{}; }

void CubeGridScene::OnUpdate(const DemoContext& ctx) {
    const float spin = m_spinSpeed;
    const float dt   = ctx.dt;
    m_world.View<TransformComponent>(
        [dt, spin](Entity, TransformComponent& t) {
            t.Rotation.y += dt * spin;
        });
}

void CubeGridScene::OnRender(const DemoContext& ctx) {
    ctx.renderSystem->Render(m_world, *ctx.camera, *ctx.objectCB, ctx.cmd, ctx.rootParamIndexCBV);
}

void CubeGridScene::OnImGui() {
    if (ImGui::SliderInt("Grid size", &m_gridSize, 1, 8))
        Rebuild();
    ImGui::SliderFloat("Spin speed", &m_spinSpeed, 0.0f, 3.0f, "%.2f");
    ImGui::Text("Cubes: %d", m_gridSize * m_gridSize);
}
