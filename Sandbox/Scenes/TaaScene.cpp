#include "Scenes/TaaScene.h"

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
struct GeoCB {
    XMFLOAT4X4 jitteredMVP;
    XMFLOAT4X4 unjitteredMVP;
    XMFLOAT4X4 prevMVP;
    XMFLOAT4X4 model;
    XMFLOAT4   color;
};
struct ResolveCB {
    float texel[2];
    int   historyValid;
    int   taaEnabled;
    float blend;
    float pad[3];
};

// Radical-inverse Halton, the standard low-discrepancy jitter sequence.
float Halton(uint32_t i, uint32_t b) {
    float f = 1.0f, r = 0.0f;
    while (i > 0) { f /= float(b); r += f * float(i % b); i /= b; }
    return r;
}

void SetTargets(ID3D12GraphicsCommandList* c, const D3D12_CPU_DESCRIPTOR_HANDLE* rtvs,
                UINT count, const D3D12_CPU_DESCRIPTOR_HANDLE* dsv, uint32_t w, uint32_t h) {
    c->OMSetRenderTargets(count, rtvs, FALSE, dsv);
    D3D12_VIEWPORT vp = { 0, 0, float(w), float(h), 0, 1 };
    D3D12_RECT     sc = { 0, 0, LONG(w), LONG(h) };
    c->RSSetViewports(1, &vp);
    c->RSSetScissorRects(1, &sc);
}
} // namespace

const char* TaaScene::Description() const {
    return "Temporal anti-aliasing. Each frame the projection is jittered a "
           "sub-pixel amount; geometry writes colour + a motion vector, and the "
           "resolve pass reprojects last frame's result along those vectors and "
           "blends it in (history-heavy), clamping to the local neighbourhood to "
           "avoid ghosting. Accumulated jitter = supersampled edges. Toggle TAA "
           "off to see the raw aliasing, and fly the camera to see reprojection.";
}

void TaaScene::OnLoad(const DemoContext& ctx) {
    XMStoreFloat4x4(&m_prevViewProj, XMMatrixIdentity());
    m_shaders.Initialize(L"Shaders");
    if (!BuildPipelines(ctx)) return;
    m_srvs.Create(ctx.device, 8);
    if (!CreateTargets(ctx, ctx.renderer->GetWidth(), ctx.renderer->GetHeight())) return;
    m_ready = true;
}

void TaaScene::OnUnload() {
    m_ready = false;
    m_color.Reset(); m_velocity.Reset(); m_historyA.Reset(); m_historyB.Reset();
    m_srvs.Reset();
    m_geoPSO.Reset(); m_resolvePSO.Reset();
    m_geoRS.Reset(); m_resolveRS.Reset();
    m_shaders.Shutdown();
}

bool TaaScene::BuildPipelines(const DemoContext& ctx) {
    ID3D12Device* dev = ctx.device;

    // Geometry -> [colour RGBA16F, velocity RG16F] + depth.
    if (!RootSignatureBuilder().Cbv(0).Build(dev, m_geoRS)) return false;
    auto gvs = m_shaders.GetOrCompile(L"TaaGeometry.hlsl", "VSMain", "vs_6_0");
    auto gps = m_shaders.GetOrCompile(L"TaaGeometry.hlsl", "PSMain", "ps_6_0");
    if (!gvs || !gps) return false;
    GraphicsPipelineDesc g; g.rootSignature = m_geoRS.Get(); g.vs = gvs; g.ps = gps;
    g.depthEnable = true; g.dsvFormat = DXGI_FORMAT_D32_FLOAT;
    g.rtvCount = 2;
    g.rtvFormats[0] = DXGI_FORMAT_R16G16B16A16_FLOAT;
    g.rtvFormats[1] = DXGI_FORMAT_R16G16_FLOAT;
    if (!m_geoPSO.Create(dev, g)) return false;

    // Resolve -> [history RGBA16F, back buffer UNORM], fullscreen, no depth.
    if (!RootSignatureBuilder().SrvTable(0, 3).Cbv(0).SamplerLinearClamp(0).Build(dev, m_resolveRS))
        return false;
    auto rvs = m_shaders.GetOrCompile(L"TaaResolve.hlsl", "VSMain", "vs_6_0");
    auto rps = m_shaders.GetOrCompile(L"TaaResolve.hlsl", "PSMain", "ps_6_0");
    if (!rvs || !rps) return false;
    GraphicsPipelineDesc r; r.rootSignature = m_resolveRS.Get(); r.vs = rvs; r.ps = rps;
    r.depthEnable = false; r.rtvCount = 2;
    r.rtvFormats[0] = DXGI_FORMAT_R16G16B16A16_FLOAT;
    r.rtvFormats[1] = DXGI_FORMAT_R8G8B8A8_UNORM;
    return m_resolvePSO.Create(dev, r);
}

bool TaaScene::CreateTargets(const DemoContext& ctx, uint32_t w, uint32_t h) {
    m_w = w; m_h = h;
    const float black[4] = { 0, 0, 0, 1 };
    const float zero[4]  = { 0, 0, 0, 0 };
    if (!m_color.Create(ctx.device, w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, black)) return false;
    if (!m_velocity.Create(ctx.device, w, h, DXGI_FORMAT_R16G16_FLOAT, zero)) return false;
    if (!m_historyA.Create(ctx.device, w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, black)) return false;
    if (!m_historyB.Create(ctx.device, w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, black)) return false;
    WriteSrvs(ctx);
    m_firstFrame = true;
    m_graph.ResetStateCache();
    return true;
}

void TaaScene::WriteSrvs(const DemoContext& ctx) {
    ID3D12Device* dev = ctx.device;
    // Two contiguous [colour, velocity, history] groups so the resolve table can
    // point at the correct ping/pong history without rewriting descriptors
    // (which could race the GPU still reading last frame's).
    m_color.CreateSrvInto(dev,    m_srvs.Cpu(0));
    m_velocity.CreateSrvInto(dev, m_srvs.Cpu(1));
    m_historyA.CreateSrvInto(dev, m_srvs.Cpu(2));
    m_color.CreateSrvInto(dev,    m_srvs.Cpu(3));
    m_velocity.CreateSrvInto(dev, m_srvs.Cpu(4));
    m_historyB.CreateSrvInto(dev, m_srvs.Cpu(5));
}

void TaaScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;
    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;

    const uint32_t w = r->GetWidth(), h = r->GetHeight();
    if (w != m_w || h != m_h) { r->WaitForGPU(); CreateTargets(ctx, w, h); }

    const XMMATRIX viewProj = ctx.camera->GetViewProjection();
    const XMMATRIX prevVP   = XMLoadFloat4x4(&m_prevViewProj);

    // Sub-pixel jitter (Halton 2,3), in NDC. Disabled when TAA is off.
    float jx = 0.0f, jy = 0.0f;
    if (m_taa) {
        jx = (Halton(m_jitterIdx + 1, 2) - 0.5f) * 2.0f / float(w);
        jy = (Halton(m_jitterIdx + 1, 3) - 0.5f) * 2.0f / float(h);
    }
    const XMMATRIX jitterVP = viewProj * XMMatrixTranslation(jx, jy, 0.0f);

    // Ping-pong: write `current`, read `prev`.
    const bool curIsA = (m_parity == 0);
    RenderTexture& curHist = curIsA ? m_historyA : m_historyB;

    m_graph.Begin(cmd);
    m_graph.SetProfiler(&r->GetProfiler());
    const RgHandle color = m_graph.Import("Color",    m_color.Resource(),    RgState::Common);
    const RgHandle vel   = m_graph.Import("Velocity", m_velocity.Resource(), RgState::Common);
    const RgHandle histA = m_graph.Import("HistoryA", m_historyA.Resource(), RgState::Common);
    const RgHandle histB = m_graph.Import("HistoryB", m_historyB.Resource(), RgState::Common);
    const RgHandle curH  = curIsA ? histA : histB;
    const RgHandle prevH = curIsA ? histB : histA;
    const RgHandle back  = m_graph.Import("BackBuffer", r->BackBufferResource(), RgState::RenderTarget);
    const RgHandle depth = m_graph.Import("SceneDepth", r->DepthResource(),      RgState::DepthWrite);

    // ---- Geometry: colour + motion vectors (jittered rasterization) ----
    m_graph.AddPass("Geometry",
        { Write(color, RgState::RenderTarget), Write(vel, RgState::RenderTarget),
          Write(depth, RgState::DepthWrite) },
        [this, r, jitterVP, viewProj, prevVP, &ctx](ID3D12GraphicsCommandList* c) {
            D3D12_CPU_DESCRIPTOR_HANDLE rtvs[2] = { m_color.Rtv(), m_velocity.Rtv() };
            D3D12_CPU_DESCRIPTOR_HANDLE dsv = r->GetDepthDSV();
            SetTargets(c, rtvs, 2, &dsv, m_w, m_h);
            m_color.ClearRtv(c); m_velocity.ClearRtv(c);
            c->SetGraphicsRootSignature(m_geoRS.Get());
            c->SetPipelineState(m_geoPSO.Get());
            c->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            const int n = 5; const float sp = 2.4f;
            const float off = (n - 1) * 0.5f * sp;
            int k = 0;
            for (int gy = 0; gy < n; ++gy)
                for (int gx = 0; gx < n; ++gx, ++k) {
                    const XMMATRIX model = XMMatrixTranslation(gx * sp - off, gy * sp - off, 0.0f);
                    GeoCB o;
                    XMStoreFloat4x4(&o.jitteredMVP,   model * jitterVP);
                    XMStoreFloat4x4(&o.unjitteredMVP, model * viewProj);
                    XMStoreFloat4x4(&o.prevMVP,       model * prevVP);
                    XMStoreFloat4x4(&o.model,         model);
                    const float hue = float(k) * 0.6f;
                    o.color = { 0.5f + 0.4f * sinf(hue), 0.5f + 0.4f * sinf(hue + 2.1f),
                                0.5f + 0.4f * sinf(hue + 4.2f), 1.0f };
                    if (ctx.objectCB->BindCbv(c, 0, o)) m_cube->Draw(c);
                }
        });

    // ---- Resolve: reproject + blend history, write new history + back buffer ----
    const uint32_t firstSlot = curIsA ? 3u : 0u; // prev = B(slots3-5) if cur=A, else A(0-2)
    m_graph.AddPass("Resolve",
        { Read(color, RgState::PixelShader), Read(vel, RgState::PixelShader),
          Read(prevH, RgState::PixelShader), Write(curH, RgState::RenderTarget),
          Write(back, RgState::RenderTarget) },
        [this, r, &curHist, firstSlot, &ctx](ID3D12GraphicsCommandList* c) {
            D3D12_CPU_DESCRIPTOR_HANDLE rtvs[2] = { curHist.Rtv(), r->GetBackBufferRTV() };
            SetTargets(c, rtvs, 2, nullptr, m_w, m_h);
            c->SetGraphicsRootSignature(m_resolveRS.Get());
            c->SetPipelineState(m_resolvePSO.Get());
            m_srvs.BindTable(c, 0, firstSlot);
            ResolveCB cb = {};
            cb.texel[0] = 1.0f / float(m_w); cb.texel[1] = 1.0f / float(m_h);
            cb.historyValid = m_firstFrame ? 0 : 1;
            cb.taaEnabled   = m_taa ? 1 : 0;
            cb.blend        = m_blend;
            ctx.objectCB->BindCbv(c, 1, cb);
            c->IASetVertexBuffers(0, 0, nullptr);
            c->DrawInstanced(3, 1, 0, 0);
        },
        /*sideEffect*/ true);

    m_graph.Execute();

    // The resolve pass left the back buffer bound as MRT slot 1 (slot 0 was the
    // history target). ImGui records into render-target slot 0, so rebind the
    // back buffer as the sole target or the UI would draw into the offscreen
    // history texture and never appear.
    r->BindBackBufferTargets(cmd);

    // Advance temporal state for next frame.
    XMStoreFloat4x4(&m_prevViewProj, viewProj);
    m_parity ^= 1;
    m_jitterIdx = (m_jitterIdx + 1) % 8;
    m_firstFrame = false;
}

void TaaScene::OnImGui() {
    ImGui::Checkbox("TAA enabled", &m_taa);
    ImGui::SliderFloat("History blend", &m_blend, 0.02f, 0.5f, "%.2f");
    ImGui::TextDisabled("Lower blend = more history = smoother, but more ghosting risk.");
    ImGui::Separator();
    ImGui::TextDisabled("Fly the camera (hold RMB + WASD) to see motion-vector reprojection.");
    ImGui::TextDisabled("Jitter: Halton(2,3), 8-sample; motion from un-jittered clip.");
}

bool TaaScene::PreferredCamera(float pos[3], float& yaw, float& pitch) const {
    pos[0] = 4.5f; pos[1] = 3.5f; pos[2] = -9.0f;
    yaw = -0.42f; pitch = -0.28f;
    return true;
}
