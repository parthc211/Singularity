#include "Scenes/RenderGraphScene.h"

#include "Scene/Components.h"
#include "Scene/RenderSystem.h"
#include "Renderer/Renderer.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"

#include "imgui.h"
#include <DirectXMath.h>
#include <cmath>

using namespace SGE;

const char* RenderGraphScene::Description() const {
    return "Drives a multi-pass frame through the RenderGraph. Instead of each "
           "pass hand-writing its resource barriers, every pass DECLARES the "
           "state it needs each resource in; the graph diffs states to emit the "
           "transitions and reference-count culls passes whose output nobody "
           "reads. The table below is the graph's actual compiled schedule this "
           "frame. Toggle 'Consume Aux target' to watch the AuxClear pass — and "
           "its barriers — get culled when its result goes unused.";
}

void RenderGraphScene::Rebuild() {
    m_world = World{};
    const int   n       = m_gridSize;
    const float spacing = 2.5f;
    const float offset  = (n - 1) * 0.5f * spacing;
    int i = 0;
    for (int gx = 0; gx < n; ++gx) {
        for (int gz = 0; gz < n; ++gz, ++i) {
            Entity e = m_world.Create();
            TransformComponent t;
            t.Position = { gx * spacing - offset, 0.0f, gz * spacing - offset };
            m_world.Add(e, t);
            MeshComponent mc; mc.MeshPtr = m_cube; m_world.Add(e, mc);
            const float k = (n * n > 1) ? float(i) / float(n * n) * DirectX::XM_2PI : 0.0f;
            MaterialComponent mat;
            mat.BaseColor = { 0.5f + 0.5f * sinf(k),
                              0.5f + 0.5f * sinf(k + DirectX::XM_2PI / 3.0f),
                              0.5f + 0.5f * sinf(k + 2.0f * DirectX::XM_2PI / 3.0f),
                              1.0f };
            m_world.Add(e, mat);
        }
    }
}

void RenderGraphScene::OnLoad(const DemoContext& ctx) {
    Rebuild();
    const float clear[4] = { 0.05f, 0.05f, 0.08f, 1.0f };
    m_aux.Create(ctx.device, ctx.renderer->GetWidth(), ctx.renderer->GetHeight(),
                 DXGI_FORMAT_R8G8B8A8_UNORM, clear);
    m_graph.ResetStateCache();
    m_ready = true;
}

void RenderGraphScene::OnUnload() {
    m_ready = false;
    m_aux.Reset();
    m_world = World{};
}

void RenderGraphScene::OnUpdate(const DemoContext& ctx) {
    const float dt = ctx.dt, spin = m_spinSpeed;
    m_world.View<TransformComponent>(
        [dt, spin](Entity, TransformComponent& t) { t.Rotation.y += dt * spin; });
}

void RenderGraphScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;

    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;

    // Keep the aux target window-sized (graph state cache is keyed by resource
    // pointer, so forget prior states when the texture is replaced).
    const uint32_t w = r->GetWidth(), h = r->GetHeight();
    if (m_aux.Width() != w || m_aux.Height() != h) {
        r->WaitForGPU();
        const float clear[4] = { 0.05f, 0.05f, 0.08f, 1.0f };
        m_aux.Create(ctx.device, w, h, DXGI_FORMAT_R8G8B8A8_UNORM, clear);
        m_graph.ResetStateCache();
    }

    m_graph.Begin(cmd);
    m_graph.SetProfiler(&r->GetProfiler());
    const RgHandle back  = m_graph.Import("BackBuffer", r->BackBufferResource(), RgState::RenderTarget);
    const RgHandle depth = m_graph.Import("SceneDepth", r->DepthResource(),      RgState::DepthWrite);
    const RgHandle aux   = m_graph.Import("Aux",        m_aux.Resource(),        RgState::Common);

    // Main scene pass. Back buffer + depth are already in RENDER_TARGET /
    // DEPTH_WRITE from Renderer::BeginFrame, so the graph correctly emits ZERO
    // barriers here — a nice check that it only transitions what actually changed.
    // sideEffect: writes the swap-chain back buffer, so it's never culled.
    m_graph.AddPass("ScenePass",
        { Write(back, RgState::RenderTarget), Write(depth, RgState::DepthWrite) },
        [this, r, &ctx](ID3D12GraphicsCommandList* c) {
            r->BindBackBufferTargets(c); // OMSet back buffer + depth, full viewport
            ctx.renderSystem->Render(m_world, *ctx.camera, *ctx.objectCB, c, ctx.rootParamIndexCBV);
        },
        /*sideEffect*/ true);

    // Auxiliary pass: clears an off-screen target. Its result is consumed only
    // when 'Consume Aux target' is on; otherwise nobody reads Aux and the graph
    // culls this whole pass (and the Common->RENDER_TARGET barrier it needed).
    m_graph.AddPass("AuxClear",
        { Write(aux, RgState::RenderTarget) },
        [this](ID3D12GraphicsCommandList* c) { m_aux.ClearRtv(c); });

    // Stand-in consumer: declares a read of Aux (forcing an automatic
    // RENDER_TARGET->PIXEL_SHADER_RESOURCE barrier and keeping AuxClear alive).
    // It performs no draw — its only job is to exist as a reader so the culling
    // behaviour is observable. A real consumer would sample Aux into the frame.
    if (m_consumeAux) {
        m_graph.AddPass("AuxConsume",
            { Read(aux, RgState::PixelShader) },
            [](ID3D12GraphicsCommandList*) { /* stand-in: no draw */ });
    }

    m_graph.Execute();
}

void RenderGraphScene::OnImGui() {
    ImGui::SliderInt("Grid size", &m_gridSize, 1, 8);
    ImGui::SameLine();
    if (ImGui::Button("Rebuild")) Rebuild();
    ImGui::SliderFloat("Spin speed", &m_spinSpeed, 0.0f, 3.0f, "%.2f");
    ImGui::Checkbox("Consume Aux target (keep AuxClear alive)", &m_consumeAux);
    ImGui::Separator();

    // The graph's actual compiled schedule for the frame just recorded.
    const auto& sched = m_graph.LastSchedule();
    ImGui::Text("Compiled schedule (%u barriers total):", m_graph.LastBarrierCount());
    if (ImGui::BeginTable("schedule", 5,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Pass");
        ImGui::TableSetupColumn("Status");
        ImGui::TableSetupColumn("Accesses");
        ImGui::TableSetupColumn("Barriers");
        ImGui::TableSetupColumn("GPU ms");
        ImGui::TableHeadersRow();
        for (const auto& p : sched) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            if (p.culled) ImGui::TextDisabled("%s", p.name.c_str());
            else          ImGui::Text("%s", p.name.c_str());
            ImGui::TableSetColumnIndex(1);
            if (p.culled)          ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.3f, 1), "CULLED");
            else if (p.sideEffect) ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1), "kept*");
            else                   ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.4f, 1), "run");
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%u", p.accesses);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%u", p.barriers);
            ImGui::TableSetColumnIndex(4);
            if (p.gpuMs >= 0.0) ImGui::Text("%.3f", p.gpuMs);
            else                ImGui::TextDisabled("-");
        }
        ImGui::EndTable();
    }
    ImGui::TextDisabled("* pinned (writes the back buffer) — never culled.");
    ImGui::TextDisabled("Uncheck the box: AuxClear loses its reader and is culled.");
}

bool RenderGraphScene::PreferredCamera(float pos[3], float& yaw, float& pitch) const {
    pos[0] = 0.0f; pos[1] = 4.0f; pos[2] = -11.0f;
    yaw = 0.0f; pitch = -0.28f;
    return true;
}
