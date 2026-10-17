#include "Scenes/SsrScene.h"

#include "Renderer/Renderer.h"
#include "Renderer/Mesh.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"
#include "Renderer/DX12/RootSignatureBuilder.h"
#include "Core/Camera.h"

#include "imgui.h"
#include <DirectXMath.h>
#include <cmath>

using namespace SGE;
using namespace DirectX;

namespace {
struct GeoCB   { XMFLOAT4X4 mvp; XMFLOAT4X4 modelView; XMFLOAT4 albedoReflect; };
struct LightCB { XMFLOAT4 viewLightDir; };
struct ResolveCB {
    XMFLOAT4X4 proj; float screen[2];
    int steps; float reach; float thickness; float strength; float edgeFade; float pad;
};

void SetTarget(ID3D12GraphicsCommandList* c, const D3D12_CPU_DESCRIPTOR_HANDLE* rtvs, UINT n,
               const D3D12_CPU_DESCRIPTOR_HANDLE* dsv, uint32_t w, uint32_t h) {
    c->OMSetRenderTargets(n, rtvs, FALSE, dsv);
    D3D12_VIEWPORT vp = { 0, 0, float(w), float(h), 0, 1 };
    D3D12_RECT     sc = { 0, 0, LONG(w), LONG(h) };
    c->RSSetViewports(1, &vp);
    c->RSSetScissorRects(1, &sc);
}
} // namespace

const char* SsrScene::Description() const {
    return "Screen-space reflections. The scene is drawn to a view-space G-buffer "
           "(colour + reflectivity, normal, position); the resolve pass reflects "
           "each pixel's view ray and marches it through the depth buffer, "
           "compositing the hit colour by reflectivity x Fresnel x edge fade. The "
           "reflective floor mirrors the objects above it. Toggle SSR off to "
           "compare, and note reflections vanish where they'd leave the screen.";
}

void SsrScene::OnLoad(const DemoContext& ctx) {
    m_shaders.Initialize(L"Shaders");
    if (!BuildPipelines(ctx)) return;
    m_srvs.Create(ctx.device, 4);
    if (!CreateTargets(ctx, ctx.renderer->GetWidth(), ctx.renderer->GetHeight())) return;
    m_ready = true;
}

void SsrScene::OnUnload() {
    m_ready = false;
    m_color.Reset(); m_normal.Reset(); m_posV.Reset();
    m_srvs.Reset();
    m_geoPSO.Reset(); m_resolvePSO.Reset();
    m_geoRS.Reset(); m_resolveRS.Reset();
    m_shaders.Shutdown();
}

bool SsrScene::BuildPipelines(const DemoContext& ctx) {
    ID3D12Device* dev = ctx.device;

    // Geometry -> [colour+reflectivity, view normal, view position] + depth.
    if (!RootSignatureBuilder().Cbv(0).Cbv(1).Build(dev, m_geoRS)) return false;
    auto gvs = m_shaders.GetOrCompile(L"SsrGeometry.hlsl", "VSMain", "vs_6_0");
    auto gps = m_shaders.GetOrCompile(L"SsrGeometry.hlsl", "PSMain", "ps_6_0");
    if (!gvs || !gps) return false;
    GraphicsPipelineDesc g; g.rootSignature = m_geoRS.Get(); g.vs = gvs; g.ps = gps;
    g.depthEnable = true; g.dsvFormat = DXGI_FORMAT_D32_FLOAT;
    g.rtvCount = 3;
    g.rtvFormats[0] = DXGI_FORMAT_R16G16B16A16_FLOAT;
    g.rtvFormats[1] = DXGI_FORMAT_R16G16B16A16_FLOAT;
    g.rtvFormats[2] = DXGI_FORMAT_R16G16B16A16_FLOAT;
    if (!m_geoPSO.Create(dev, g)) return false;

    // Resolve -> back buffer.
    if (!RootSignatureBuilder().SrvTable(0, 3).Cbv(0).SamplerLinearClamp(0).Build(dev, m_resolveRS))
        return false;
    auto rvs = m_shaders.GetOrCompile(L"SsrResolve.hlsl", "VSMain", "vs_6_0");
    auto rps = m_shaders.GetOrCompile(L"SsrResolve.hlsl", "PSMain", "ps_6_0");
    if (!rvs || !rps) return false;
    GraphicsPipelineDesc r; r.rootSignature = m_resolveRS.Get(); r.vs = rvs; r.ps = rps;
    r.depthEnable = false; r.rtvFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
    return m_resolvePSO.Create(dev, r);
}

bool SsrScene::CreateTargets(const DemoContext& ctx, uint32_t w, uint32_t h) {
    m_w = w; m_h = h;
    const float black[4] = { 0, 0, 0, 0 };
    if (!m_color.Create(ctx.device, w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, black)) return false;
    if (!m_normal.Create(ctx.device, w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, black)) return false;
    if (!m_posV.Create(ctx.device, w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, black)) return false;
    WriteSrvs(ctx);
    m_graph.ResetStateCache();
    return true;
}

void SsrScene::WriteSrvs(const DemoContext& ctx) {
    m_color.CreateSrvInto(ctx.device,  m_srvs.Cpu(0));
    m_normal.CreateSrvInto(ctx.device, m_srvs.Cpu(1));
    m_posV.CreateSrvInto(ctx.device,   m_srvs.Cpu(2));
}

void SsrScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;
    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;

    const uint32_t w = r->GetWidth(), h = r->GetHeight();
    if (w != m_w || h != m_h) { r->WaitForGPU(); CreateTargets(ctx, w, h); }

    const XMMATRIX view     = ctx.camera->GetView();
    const XMMATRIX viewProj = ctx.camera->GetViewProjection();

    // World light -> view space, so shading is stable as the camera moves.
    const XMVECTOR worldLight = XMVector3Normalize(XMVectorSet(0.35f, -0.75f, 0.4f, 0.0f));
    XMFLOAT4 viewLight;
    XMStoreFloat4(&viewLight, XMVector3Normalize(XMVector3TransformNormal(worldLight, view)));

    m_graph.Begin(cmd);
    m_graph.SetProfiler(&r->GetProfiler());
    const RgHandle color = m_graph.Import("Color",  m_color.Resource(),  RgState::Common);
    const RgHandle norm  = m_graph.Import("Normal", m_normal.Resource(), RgState::Common);
    const RgHandle posv  = m_graph.Import("PosV",   m_posV.Resource(),   RgState::Common);
    const RgHandle back  = m_graph.Import("BackBuffer", r->BackBufferResource(), RgState::RenderTarget);
    const RgHandle depth = m_graph.Import("SceneDepth", r->DepthResource(),      RgState::DepthWrite);

    // ---- Geometry -> view-space G-buffer ----
    m_graph.AddPass("Geometry",
        { Write(color, RgState::RenderTarget), Write(norm, RgState::RenderTarget),
          Write(posv, RgState::RenderTarget), Write(depth, RgState::DepthWrite) },
        [this, r, view, viewProj, viewLight, &ctx](ID3D12GraphicsCommandList* c) {
            D3D12_CPU_DESCRIPTOR_HANDLE rtvs[3] = { m_color.Rtv(), m_normal.Rtv(), m_posV.Rtv() };
            D3D12_CPU_DESCRIPTOR_HANDLE dsv = r->GetDepthDSV();
            SetTarget(c, rtvs, 3, &dsv, m_w, m_h);
            m_color.ClearRtv(c); m_normal.ClearRtv(c); m_posV.ClearRtv(c);
            c->SetGraphicsRootSignature(m_geoRS.Get());
            c->SetPipelineState(m_geoPSO.Get());
            c->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            LightCB lc{ viewLight }; ctx.objectCB->BindCbv(c, 1, lc);

            auto draw = [&](Mesh* m, XMMATRIX model, XMFLOAT4 albedoReflect) {
                GeoCB o; XMStoreFloat4x4(&o.mvp, model * viewProj);
                XMStoreFloat4x4(&o.modelView, model * view); o.albedoReflect = albedoReflect;
                ctx.objectCB->BindCbv(c, 0, o); m->Draw(c);
            };
            // Reflective floor (cube.obj is half-extent 0.5, so a 0.2 Y scale -> top at -1.1).
            const float floorTop = -1.1f;
            draw(m_cube, XMMatrixScaling(40.0f, 0.2f, 40.0f) * XMMatrixTranslation(0, -1.2f, 0),
                 { 0.35f, 0.37f, 0.4f, 1.0f });
            // Colored objects sitting ON the floor. cube.obj is half-extent 0.5;
            // the procedural sphere is radius 1.0 — so each object's centre is
            // placed at floorTop + its own half-height, or it floats and its
            // reflection detaches with a gap.
            const float cols[5][3] = { {0.9f,0.3f,0.3f},{0.3f,0.8f,0.4f},{0.35f,0.5f,0.95f},
                                       {0.95f,0.8f,0.3f},{0.8f,0.4f,0.85f} };
            for (int k = 0; k < 5; ++k) {
                const float x   = (k - 2) * 3.2f;
                const bool  sph = (k & 1) != 0;
                const float scl = sph ? 1.0f : 1.5f;
                const float half = sph ? scl : 0.5f * scl;   // sphere radius vs cube half-extent
                XMFLOAT4 col{ cols[k][0], cols[k][1], cols[k][2], 0.1f };
                Mesh* mesh = sph ? m_sphere : m_cube;
                draw(mesh, XMMatrixScaling(scl, scl, scl) *
                           XMMatrixTranslation(x, floorTop + half, 2.0f), col);
            }
        });

    // ---- Resolve: ray-march reflections, composite to back buffer ----
    m_graph.AddPass("SSR Resolve",
        { Read(color, RgState::PixelShader), Read(norm, RgState::PixelShader),
          Read(posv, RgState::PixelShader), Write(back, RgState::RenderTarget) },
        [this, r, &ctx](ID3D12GraphicsCommandList* c) {
            D3D12_CPU_DESCRIPTOR_HANDLE rtv = r->GetBackBufferRTV();
            SetTarget(c, &rtv, 1, nullptr, m_w, m_h);
            c->SetGraphicsRootSignature(m_resolveRS.Get());
            c->SetPipelineState(m_resolvePSO.Get());
            m_srvs.BindTable(c, 0, 0);
            ResolveCB cb{};
            XMStoreFloat4x4(&cb.proj, ctx.camera->GetProjection());
            cb.screen[0] = float(m_w); cb.screen[1] = float(m_h);
            cb.steps = m_steps; cb.reach = m_reach; cb.thickness = m_thickness;
            cb.strength = m_enabled ? m_strength : 0.0f; cb.edgeFade = m_edgeFade;
            ctx.objectCB->BindCbv(c, 1, cb);
            c->IASetVertexBuffers(0, 0, nullptr);
            c->DrawInstanced(3, 1, 0, 0);
        },
        /*sideEffect*/ true);

    m_graph.Execute();
}

void SsrScene::OnImGui() {
    ImGui::Checkbox("SSR enabled", &m_enabled);
    ImGui::SliderFloat("Strength", &m_strength, 0.0f, 1.0f, "%.2f");
    ImGui::SliderInt("March steps", &m_steps, 16, 160);
    ImGui::SliderFloat("Ray reach", &m_reach, 5.0f, 60.0f, "%.0f");
    ImGui::SliderFloat("Thickness", &m_thickness, 0.05f, 2.0f, "%.2f");
    ImGui::SliderFloat("Edge fade", &m_edgeFade, 0.01f, 0.4f, "%.2f");
    ImGui::TextDisabled("Reflective floor mirrors the objects; reflections fade");
    ImGui::TextDisabled("where the ray would leave the screen (SSR's limit).");
}

bool SsrScene::PreferredCamera(float pos[3], float& yaw, float& pitch) const {
    pos[0] = 0.0f; pos[1] = 2.6f; pos[2] = -9.5f;
    yaw = 0.0f; pitch = -0.24f;
    return true;
}
