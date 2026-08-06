#include "Scenes/BloomScene.h"

#include "Core/Camera.h"
#include "Renderer/Renderer.h"
#include "Renderer/Mesh.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"
#include "Renderer/DX12/RootSignatureBuilder.h"

#include "imgui.h"
#include <DirectXMath.h>
#include <cmath>

using namespace SGE;
using namespace DirectX;

namespace {
constexpr DXGI_FORMAT kHDR = DXGI_FORMAT_R16G16B16A16_FLOAT;

struct GeoObjCB { XMFLOAT4X4 MVP; XMFLOAT4X4 Model; XMFLOAT4 Color; };
struct PostCB   { XMFLOAT2 TexelSize; float Direction; float Threshold; float Intensity; float Exposure; XMFLOAT2 _pad; };
}

const char* BloomScene::Description() const {
    return "HDR + bloom: emissive shapes (colours > 1) are rendered into a "
           "floating-point target; a bright-pass extracts the glow, a separable "
           "Gaussian blur spreads it, and a final pass adds it back and ACES-"
           "tonemaps to the 8-bit back buffer. Tune threshold / intensity / "
           "exposure and the blur-pass count below.";
}

void BloomScene::BuildObjects() {
    m_objects.clear();
    // A ring of cubes, alternating bright (emissive) and dim.
    const int n = 8;
    for (int i = 0; i < n; ++i) {
        const float a = (float)i / n * XM_2PI;
        const bool  bright = (i % 2) == 0;
        XMFLOAT4 col = bright
            ? XMFLOAT4(2.6f * (0.6f + 0.4f * sinf(a)),       2.4f, 0.7f, 1) // hot
            : XMFLOAT4(0.35f, 0.40f, 0.55f, 1);                            // dim
        if (bright) col = XMFLOAT4(2.8f, 1.2f + 1.4f * (i / (float)n), 0.5f, 1);
        m_objects.push_back({ a, 5.0f, 0.0f, 0.9f, (bright ? 1.5f : 0.8f), col });
    }
    // Bright centerpiece.
    m_objects.push_back({ 0.0f, 0.0f, 0.0f, 1.4f, 0.6f, XMFLOAT4(3.5f, 3.2f, 3.0f, 1) });
}

bool BloomScene::BuildPipelines(const DemoContext& ctx) {
    ID3D12Device* device = ctx.device;
    m_shaders.Initialize(L"Shaders");

    // --- Geometry root sig: one per-object CBV. ---
    if (!RootSignatureBuilder().Cbv(0).Build(device, m_geoRootSig)) return false;

    // Post root sigs: SRV table (param 0) + pixel CBV b0 (param 1), linear-clamp sampler.
    auto makePostRootSig = [&](RootSignature& rs, UINT srvCount) {
        return RootSignatureBuilder()
            .SrvTable(0, srvCount)
            .Cbv(0, D3D12_SHADER_VISIBILITY_PIXEL)
            .SamplerLinearClamp(0)
            .Build(device, rs);
    };
    if (!makePostRootSig(m_postRootSig, 1))      return false; // bright / blur: 1 input
    if (!makePostRootSig(m_compositeRootSig, 2)) return false; // composite: scene + bloom

    auto gvs = m_shaders.GetOrCompile(L"BloomGeometry.hlsl", "VSMain", "vs_6_0");
    auto gps = m_shaders.GetOrCompile(L"BloomGeometry.hlsl", "PSMain", "ps_6_0");
    auto bps = m_shaders.GetOrCompile(L"BloomBright.hlsl", "PSMain", "ps_6_0");
    auto bvs = m_shaders.GetOrCompile(L"BloomBright.hlsl", "VSMain", "vs_6_0");
    auto blvs = m_shaders.GetOrCompile(L"BloomBlur.hlsl", "VSMain", "vs_6_0");
    auto blps = m_shaders.GetOrCompile(L"BloomBlur.hlsl", "PSMain", "ps_6_0");
    auto cvs = m_shaders.GetOrCompile(L"BloomComposite.hlsl", "VSMain", "vs_6_0");
    auto cps = m_shaders.GetOrCompile(L"BloomComposite.hlsl", "PSMain", "ps_6_0");
    if (!gvs || !gps || !bps || !bvs || !blvs || !blps || !cvs || !cps) return false;

    GraphicsPipelineDesc geo;
    geo.rootSignature = m_geoRootSig.Get(); geo.vs = gvs; geo.ps = gps;
    geo.depthEnable = true; geo.rtvFormat = kHDR;
    if (!m_geoPSO.Create(device, geo)) return false;

    GraphicsPipelineDesc bright;
    bright.rootSignature = m_postRootSig.Get(); bright.vs = bvs; bright.ps = bps;
    bright.depthEnable = false; bright.rtvFormat = kHDR;
    if (!m_brightPSO.Create(device, bright)) return false;

    GraphicsPipelineDesc blur;
    blur.rootSignature = m_postRootSig.Get(); blur.vs = blvs; blur.ps = blps;
    blur.depthEnable = false; blur.rtvFormat = kHDR;
    if (!m_blurPSO.Create(device, blur)) return false;

    GraphicsPipelineDesc comp;
    comp.rootSignature = m_compositeRootSig.Get(); comp.vs = cvs; comp.ps = cps;
    comp.depthEnable = false; comp.rtvFormat = DXGI_FORMAT_R8G8B8A8_UNORM; // back buffer
    return m_compositePSO.Create(device, comp);
}

bool BloomScene::CreateTargets(ID3D12Device* device, uint32_t w, uint32_t h) {
    m_targetW = w; m_targetH = h;

    if (!m_srvs.IsValid()) {
        if (!m_srvs.Create(device, 3)) return false; // scene, bloomA, bloomB
    }

    const float dark[4]  = { 0.02f, 0.02f, 0.03f, 1.0f };
    const float black[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
    if (!m_sceneHDR.Create(device, w, h, kHDR, dark))  return false;
    if (!m_bloomA.Create(device, w, h, kHDR, black))   return false;
    if (!m_bloomB.Create(device, w, h, kHDR, black))   return false;

    m_srvs.Write(device, 0, m_sceneHDR);
    m_srvs.Write(device, 1, m_bloomA);
    m_srvs.Write(device, 2, m_bloomB);
    return true;
}

void BloomScene::OnLoad(const DemoContext& ctx) {
    BuildObjects();
    const bool pl = BuildPipelines(ctx);
    const bool tg = CreateTargets(ctx.device, ctx.renderer->GetWidth(), ctx.renderer->GetHeight());
    m_ready = pl && tg;
}

void BloomScene::OnUnload() {
    m_sceneHDR.Reset(); m_bloomA.Reset(); m_bloomB.Reset();
    m_srvs.Reset(); m_targetW = m_targetH = 0;
    m_geoPSO.Reset(); m_brightPSO.Reset(); m_blurPSO.Reset(); m_compositePSO.Reset();
    m_geoRootSig.Reset(); m_postRootSig.Reset(); m_compositeRootSig.Reset();
    m_shaders.Shutdown();
    m_objects.clear();
    m_ready = false;
}

void BloomScene::OnUpdate(const DemoContext& ctx) {
    if (m_animate) m_time += ctx.dt;
}

void BloomScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;
    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;

    const uint32_t w = r->GetWidth(), h = r->GetHeight();
    if (w != m_targetW || h != m_targetH) { r->WaitForGPU(); CreateTargets(ctx.device, w, h); }

    const XMMATRIX camVP = ctx.camera->GetViewProjection();
    const XMFLOAT2 texel = { 1.0f / w, 1.0f / h };

    auto fullscreen = [&] { cmd->DrawInstanced(3, 1, 0, 0); }; // fullscreen triangle (SV_VertexID)
    auto post = [&](float dir, float thr, float intensity, float exposure) {
        PostCB cb{ texel, dir, thr, intensity, exposure, {0,0} };
        ctx.objectCB->BindCbv(cmd, 1, cb);
    };

    // ---- Geometry -> HDR scene target ----
    m_sceneHDR.TransitionTo(cmd, D3D12_RESOURCE_STATE_RENDER_TARGET);
    D3D12_CPU_DESCRIPTOR_HANDLE hdrRtv = m_sceneHDR.Rtv();
    D3D12_CPU_DESCRIPTOR_HANDLE depthDsv = r->GetDepthDSV();
    cmd->OMSetRenderTargets(1, &hdrRtv, FALSE, &depthDsv);
    m_sceneHDR.ClearRtv(cmd); // depth cleared by Renderer::BeginFrame
    cmd->SetGraphicsRootSignature(m_geoRootSig.Get());
    cmd->SetPipelineState(m_geoPSO.Get());
    cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    for (const Obj& o : m_objects) {
        const float orbit = o.baseAngle + m_time * 0.3f;
        XMMATRIX model = XMMatrixScaling(o.scale, o.scale, o.scale)
                       * XMMatrixRotationY(m_time * o.spin)
                       * XMMatrixTranslation(cosf(orbit) * o.radius, o.y, sinf(orbit) * o.radius);
        GeoObjCB cb;
        XMStoreFloat4x4(&cb.MVP, model * camVP);
        XMStoreFloat4x4(&cb.Model, model);
        cb.Color = o.color;
        if (!ctx.objectCB->BindCbv(cmd, 0, cb)) continue;
        m_cube->Draw(cmd);
    }

    // All post passes sample through the shared SRV heap.
    cmd->IASetVertexBuffers(0, 0, nullptr); // fullscreen passes use no vertex buffer

    // ---- Bright-pass: scene -> bloomA ----
    m_sceneHDR.TransitionTo(cmd, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    m_bloomA.TransitionTo(cmd, D3D12_RESOURCE_STATE_RENDER_TARGET);
    D3D12_CPU_DESCRIPTOR_HANDLE rtv = m_bloomA.Rtv();
    cmd->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
    cmd->SetGraphicsRootSignature(m_postRootSig.Get());
    cmd->SetPipelineState(m_brightPSO.Get());
    m_srvs.BindTable(cmd, 0, 0); // scene
    post(0.0f, m_threshold, 0.0f, 0.0f);
    fullscreen();

    // ---- Separable Gaussian blur, ping-pong (result ends in bloomA) ----
    cmd->SetPipelineState(m_blurPSO.Get());
    for (int i = 0; i < m_blurPasses; ++i) {
        // Horizontal: bloomA -> bloomB
        m_bloomA.TransitionTo(cmd, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        m_bloomB.TransitionTo(cmd, D3D12_RESOURCE_STATE_RENDER_TARGET);
        rtv = m_bloomB.Rtv(); cmd->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
        m_srvs.BindTable(cmd, 0, 1); // bloomA
        post(0.0f, 0.0f, 0.0f, 0.0f);
        fullscreen();
        // Vertical: bloomB -> bloomA
        m_bloomB.TransitionTo(cmd, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        m_bloomA.TransitionTo(cmd, D3D12_RESOURCE_STATE_RENDER_TARGET);
        rtv = m_bloomA.Rtv(); cmd->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
        m_srvs.BindTable(cmd, 0, 2); // bloomB
        post(1.0f, 0.0f, 0.0f, 0.0f);
        fullscreen();
    }

    // ---- Composite: scene + bloomA -> back buffer (tonemap) ----
    m_bloomA.TransitionTo(cmd, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    D3D12_CPU_DESCRIPTOR_HANDLE backRtv = r->GetBackBufferRTV();
    cmd->OMSetRenderTargets(1, &backRtv, FALSE, nullptr);
    cmd->SetGraphicsRootSignature(m_compositeRootSig.Get());
    cmd->SetPipelineState(m_compositePSO.Get());
    m_srvs.BindTable(cmd, 0, 0); // scene(t0) + bloomA(t1)
    post(0.0f, 0.0f, m_intensity, m_exposure);
    fullscreen();
}

void BloomScene::OnImGui() {
    ImGui::Checkbox("Animate", &m_animate);
    ImGui::SliderFloat("Bright threshold", &m_threshold, 0.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("Bloom intensity",  &m_intensity, 0.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("Exposure",         &m_exposure, 0.1f, 3.0f, "%.2f");
    ImGui::SliderInt("Blur passes",        &m_blurPasses, 1, 6);
    ImGui::TextDisabled("HDR RGBA16F target -> bright-pass -> %d x (H+V blur) -> ACES tonemap", m_blurPasses);
}
