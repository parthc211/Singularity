#include "Scenes/PostProcessScene.h"

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
struct GeoCB      { XMFLOAT4X4 mvp; XMFLOAT4X4 model; XMFLOAT4 albedo; XMFLOAT4 emissive; };
struct BrightCB   { float threshold; float knee; float pad[2]; };
struct BlurCB     { float texelDir[2]; float pad[2]; };
struct CompositeCB {
    float exposure; int autoExposure; float bloomStrength; float vignette;
    float ca; float grain; float contrast; float saturation;
    float temperature; float keyValue; float time; int gamma;
};
struct ExposureConsts { uint32_t dim[2]; float adaptSpeed; float dt; };

// Linear-clamp sampler visible to ALL stages (presets are pixel-only; the
// compute exposure pass needs it).
D3D12_STATIC_SAMPLER_DESC LinearClampAll(UINT reg) {
    D3D12_STATIC_SAMPLER_DESC s = {};
    s.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    s.AddressU = s.AddressV = s.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    s.MaxLOD = D3D12_FLOAT32_MAX;
    s.ShaderRegister = reg;
    s.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    return s;
}

void SetTarget(ID3D12GraphicsCommandList* c, D3D12_CPU_DESCRIPTOR_HANDLE rtv,
               const D3D12_CPU_DESCRIPTOR_HANDLE* dsv, uint32_t w, uint32_t h) {
    c->OMSetRenderTargets(1, &rtv, FALSE, dsv);
    D3D12_VIEWPORT vp = { 0, 0, float(w), float(h), 0, 1 };
    D3D12_RECT     sc = { 0, 0, LONG(w), LONG(h) };
    c->RSSetViewports(1, &vp);
    c->RSSetScissorRects(1, &sc);
}
} // namespace

const char* PostProcessScene::Description() const {
    return "A composable post-processing stack over an HDR target, run through "
           "the render graph: eye-adaptation (compute), bloom (bright-pass + "
           "separable Gaussian), then a composite doing ACES tonemap, colour "
           "grade, vignette, chromatic aberration and film grain. Every effect "
           "is a toggle below — the stack builds up one node at a time.";
}

void PostProcessScene::OnLoad(const DemoContext& ctx) {
    m_shaders.Initialize(L"Shaders");
    if (!BuildPipelines(ctx))
        return;
    m_srvs.Create(ctx.device, 8);
    if (!CreateTargets(ctx, ctx.renderer->GetWidth(), ctx.renderer->GetHeight()))
        return;
    m_ready = true;
}

void PostProcessScene::OnUnload() {
    m_ready = false;
    m_hdr.Reset(); m_bloomA.Reset(); m_bloomB.Reset(); m_adapted.Reset();
    m_srvs.Reset();
    m_geoPSO.Reset(); m_brightPSO.Reset(); m_blurPSO.Reset(); m_compositePSO.Reset();
    m_exposurePSO.Reset();
    m_geoRS.Reset(); m_brightRS.Reset(); m_blurRS.Reset(); m_exposureRS.Reset(); m_compositeRS.Reset();
    m_shaders.Shutdown();
}

bool PostProcessScene::BuildPipelines(const DemoContext& ctx) {
    ID3D12Device* dev = ctx.device;
    auto fs = [&](const wchar_t* f, const char* e) { return m_shaders.GetOrCompile(f, e, "ps_6_0"); };
    auto vsf = [&](const wchar_t* f, const char* e) { return m_shaders.GetOrCompile(f, e, "vs_6_0"); };

    // Geometry -> HDR.
    if (!RootSignatureBuilder().Cbv(0).Build(dev, m_geoRS)) return false;
    { GraphicsPipelineDesc d; d.rootSignature = m_geoRS.Get();
      d.vs = vsf(L"PostGeometry.hlsl", "VSMain"); d.ps = fs(L"PostGeometry.hlsl", "PSMain");
      if (!d.vs || !d.ps) return false;
      d.depthEnable = true; d.dsvFormat = DXGI_FORMAT_D32_FLOAT;
      d.rtvFormat = DXGI_FORMAT_R16G16B16A16_FLOAT;
      if (!m_geoPSO.Create(dev, d)) return false; }

    // Bright-pass (fullscreen, no depth, HDR out).
    if (!RootSignatureBuilder().SrvTable(0, 1).Cbv(0).SamplerLinearClamp(0).Build(dev, m_brightRS)) return false;
    { GraphicsPipelineDesc d; d.rootSignature = m_brightRS.Get();
      d.vs = vsf(L"PostBright.hlsl", "VSMain"); d.ps = fs(L"PostBright.hlsl", "PSMain");
      if (!d.vs || !d.ps) return false;
      d.depthEnable = false; d.rtvFormat = DXGI_FORMAT_R16G16B16A16_FLOAT;
      if (!m_brightPSO.Create(dev, d)) return false; }

    // Separable blur.
    if (!RootSignatureBuilder().SrvTable(0, 1).Cbv(0).SamplerLinearClamp(0).Build(dev, m_blurRS)) return false;
    { GraphicsPipelineDesc d; d.rootSignature = m_blurRS.Get();
      d.vs = vsf(L"PostBlur.hlsl", "VSMain"); d.ps = fs(L"PostBlur.hlsl", "PSMain");
      if (!d.vs || !d.ps) return false;
      d.depthEnable = false; d.rtvFormat = DXGI_FORMAT_R16G16B16A16_FLOAT;
      if (!m_blurPSO.Create(dev, d)) return false; }

    // Composite -> back buffer.
    if (!RootSignatureBuilder().SrvTable(0, 3).Cbv(0).SamplerLinearClamp(0).Build(dev, m_compositeRS)) return false;
    { GraphicsPipelineDesc d; d.rootSignature = m_compositeRS.Get();
      d.vs = vsf(L"PostComposite.hlsl", "VSMain"); d.ps = fs(L"PostComposite.hlsl", "PSMain");
      if (!d.vs || !d.ps) return false;
      d.depthEnable = false; d.rtvFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
      if (!m_compositePSO.Create(dev, d)) return false; }

    // Exposure (compute): [0] constants, [1] scene SRV, [2] adapted UAV.
    if (!RootSignatureBuilder().Constants(0, 4).SrvTable(0, 1, D3D12_SHADER_VISIBILITY_ALL)
             .UavTable(0, 1).Sampler(LinearClampAll(0)).Build(dev, m_exposureRS,
                                                              D3D12_ROOT_SIGNATURE_FLAG_NONE))
        return false;
    auto cs = m_shaders.GetOrCompile(L"PostExposure.hlsl", "CSMain", "cs_6_0");
    if (!cs || !m_exposurePSO.Create(dev, m_exposureRS.Get(), cs)) return false;
    return true;
}

bool PostProcessScene::CreateTargets(const DemoContext& ctx, uint32_t w, uint32_t h) {
    m_w = w; m_h = h; m_bw = (w + 1) / 2; m_bh = (h + 1) / 2;
    const float black[4] = { 0, 0, 0, 1 };
    if (!m_hdr.Create(ctx.device, w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, black)) return false;
    if (!m_bloomA.Create(ctx.device, m_bw, m_bh, DXGI_FORMAT_R16G16B16A16_FLOAT, black)) return false;
    if (!m_bloomB.Create(ctx.device, m_bw, m_bh, DXGI_FORMAT_R16G16B16A16_FLOAT, black)) return false;

    // 1x1 adapted-luminance target (UAV written by exposure, SRV read by composite).
    D3D12_HEAP_PROPERTIES heap = {}; heap.Type = D3D12_HEAP_TYPE_DEFAULT;
    D3D12_RESOURCE_DESC d = {};
    d.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    d.Width = 1; d.Height = 1; d.DepthOrArraySize = 1; d.MipLevels = 1;
    d.Format = DXGI_FORMAT_R32_FLOAT; d.SampleDesc.Count = 1;
    d.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
    m_adapted.Reset();
    if (FAILED(ctx.device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &d,
            D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&m_adapted))))
        return false;

    WriteSrvs(ctx);
    m_graph.ResetStateCache();
    return true;
}

void PostProcessScene::WriteSrvs(const DemoContext& ctx) {
    ID3D12Device* dev = ctx.device;
    m_hdr.CreateSrvInto(dev,    m_srvs.Cpu(0));
    m_bloomA.CreateSrvInto(dev, m_srvs.Cpu(1));
    m_bloomB.CreateSrvInto(dev, m_srvs.Cpu(2));
    m_hdr.CreateSrvInto(dev,    m_srvs.Cpu(3)); // composite input 0 (contiguous 3,4,5)
    m_bloomA.CreateSrvInto(dev, m_srvs.Cpu(4)); // composite input 1
    // adapted SRV (slot 5) + UAV (slot 6)
    D3D12_SHADER_RESOURCE_VIEW_DESC srv = {};
    srv.Format = DXGI_FORMAT_R32_FLOAT; srv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    srv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING; srv.Texture2D.MipLevels = 1;
    dev->CreateShaderResourceView(m_adapted.Get(), &srv, m_srvs.Cpu(5));
    D3D12_UNORDERED_ACCESS_VIEW_DESC uav = {};
    uav.Format = DXGI_FORMAT_R32_FLOAT; uav.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    dev->CreateUnorderedAccessView(m_adapted.Get(), nullptr, &uav, m_srvs.Cpu(6));
}

void PostProcessScene::OnUpdate(const DemoContext& ctx) { m_time += ctx.dt; }

void PostProcessScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;
    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;

    const uint32_t w = r->GetWidth(), h = r->GetHeight();
    if (w != m_w || h != m_h) { r->WaitForGPU(); CreateTargets(ctx, w, h); }

    const XMMATRIX viewProj = ctx.camera->GetViewProjection();

    m_graph.Begin(cmd);
    m_graph.SetProfiler(&r->GetProfiler());
    const RgHandle hdr   = m_graph.Import("HDR",      m_hdr.Resource(),    RgState::Common);
    const RgHandle bA    = m_graph.Import("BloomA",   m_bloomA.Resource(), RgState::Common);
    const RgHandle bB    = m_graph.Import("BloomB",   m_bloomB.Resource(), RgState::Common);
    const RgHandle adapt = m_graph.Import("Adapted",  m_adapted.Get(),     RgState::Common);
    const RgHandle back  = m_graph.Import("BackBuffer", r->BackBufferResource(), RgState::RenderTarget);
    const RgHandle depth = m_graph.Import("SceneDepth", r->DepthResource(),      RgState::DepthWrite);

    // ---- Scene -> HDR ----
    m_graph.AddPass("Scene",
        { Write(hdr, RgState::RenderTarget), Write(depth, RgState::DepthWrite) },
        [this, r, viewProj, &ctx](ID3D12GraphicsCommandList* c) {
            D3D12_CPU_DESCRIPTOR_HANDLE dsv = r->GetDepthDSV();
            SetTarget(c, m_hdr.Rtv(), &dsv, m_w, m_h);
            m_hdr.ClearRtv(c);
            c->SetGraphicsRootSignature(m_geoRS.Get());
            c->SetPipelineState(m_geoPSO.Get());
            c->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            const int nx = 7, ny = 4; const float sp = 2.6f;
            const float ox = (nx - 1) * 0.5f * sp, oy = (ny - 1) * 0.5f * sp;
            int k = 0;
            for (int gy = 0; gy < ny; ++gy)
                for (int gx = 0; gx < nx; ++gx, ++k) {
                    const XMMATRIX model = XMMatrixScaling(0.95f, 0.95f, 0.95f) *
                        XMMatrixTranslation(gx * sp - ox, gy * sp - oy, 0.0f);
                    GeoCB o = {};
                    XMStoreFloat4x4(&o.mvp, model * viewProj);
                    XMStoreFloat4x4(&o.model, model);
                    const bool emit = ((gx + gy) % 3) == 0;
                    o.albedo = { 0.5f, 0.55f, 0.6f, 1.0f };
                    if (emit) {
                        const float hue = float(k) * 0.9f;
                        o.emissive = { 3.0f + 3.0f * (0.5f + 0.5f * sinf(hue)),
                                       3.0f + 3.0f * (0.5f + 0.5f * sinf(hue + 2.1f)),
                                       3.0f + 3.0f * (0.5f + 0.5f * sinf(hue + 4.2f)), 0 };
                    }
                    if (ctx.objectCB->BindCbv(c, 0, o)) m_sphere->Draw(c);
                }
        });

    // ---- Eye adaptation (compute): HDR -> 1x1 adapted luminance ----
    m_graph.AddPass("Exposure",
        { Read(hdr, RgState::NonPixelShader), Write(adapt, RgState::UnorderedAccess) },
        [this, &ctx](ID3D12GraphicsCommandList* c) {
            ID3D12DescriptorHeap* heaps[] = { m_srvs.Heap() };
            c->SetDescriptorHeaps(1, heaps);
            c->SetComputeRootSignature(m_exposureRS.Get());
            c->SetPipelineState(m_exposurePSO.Get());
            ExposureConsts e = { { m_w, m_h }, m_adaptSpeed, ctx.dt };
            c->SetComputeRoot32BitConstants(0, 4, &e, 0);
            c->SetComputeRootDescriptorTable(1, m_srvs.Gpu(0)); // scene
            c->SetComputeRootDescriptorTable(2, m_srvs.Gpu(6)); // adapted UAV
            c->Dispatch(1, 1, 1);
        });

    // ---- Bloom bright-pass: HDR -> bloomA (half res) ----
    m_graph.AddPass("Bright",
        { Read(hdr, RgState::PixelShader), Write(bA, RgState::RenderTarget) },
        [this, &ctx](ID3D12GraphicsCommandList* c) {
            SetTarget(c, m_bloomA.Rtv(), nullptr, m_bw, m_bh);
            c->SetGraphicsRootSignature(m_brightRS.Get());
            c->SetPipelineState(m_brightPSO.Get());
            m_srvs.BindTable(c, 0, 0); // scene
            BrightCB b = { m_threshold, 0.5f, { 0, 0 } };
            ctx.objectCB->BindCbv(c, 1, b);
            c->IASetVertexBuffers(0, 0, nullptr);
            c->DrawInstanced(3, 1, 0, 0);
        });

    // ---- Separable blur ping-pong (bloomA <-> bloomB) ----
    for (int i = 0; i < m_blurIters; ++i) {
        m_graph.AddPass("BlurH",
            { Read(bA, RgState::PixelShader), Write(bB, RgState::RenderTarget) },
            [this, &ctx](ID3D12GraphicsCommandList* c) {
                SetTarget(c, m_bloomB.Rtv(), nullptr, m_bw, m_bh);
                c->SetGraphicsRootSignature(m_blurRS.Get());
                c->SetPipelineState(m_blurPSO.Get());
                m_srvs.BindTable(c, 0, 1); // bloomA
                BlurCB b = { { 1.0f / float(m_bw), 0.0f }, { 0, 0 } };
                ctx.objectCB->BindCbv(c, 1, b);
                c->IASetVertexBuffers(0, 0, nullptr);
                c->DrawInstanced(3, 1, 0, 0);
            });
        m_graph.AddPass("BlurV",
            { Read(bB, RgState::PixelShader), Write(bA, RgState::RenderTarget) },
            [this, &ctx](ID3D12GraphicsCommandList* c) {
                SetTarget(c, m_bloomA.Rtv(), nullptr, m_bw, m_bh);
                c->SetGraphicsRootSignature(m_blurRS.Get());
                c->SetPipelineState(m_blurPSO.Get());
                m_srvs.BindTable(c, 0, 2); // bloomB
                BlurCB b = { { 0.0f, 1.0f / float(m_bh) }, { 0, 0 } };
                ctx.objectCB->BindCbv(c, 1, b);
                c->IASetVertexBuffers(0, 0, nullptr);
                c->DrawInstanced(3, 1, 0, 0);
            });
    }

    // ---- Composite -> back buffer ----
    m_graph.AddPass("Composite",
        { Read(hdr, RgState::PixelShader), Read(bA, RgState::PixelShader),
          Read(adapt, RgState::PixelShader), Write(back, RgState::RenderTarget) },
        [this, r, &ctx](ID3D12GraphicsCommandList* c) {
            D3D12_CPU_DESCRIPTOR_HANDLE rtv = r->GetBackBufferRTV();
            SetTarget(c, rtv, nullptr, m_w, m_h);
            c->SetGraphicsRootSignature(m_compositeRS.Get());
            c->SetPipelineState(m_compositePSO.Get());
            m_srvs.BindTable(c, 0, 3); // hdr, bloomA, adapted (slots 3,4,5)
            CompositeCB cb = {};
            cb.exposure = m_exposure; cb.autoExposure = m_autoExposure ? 1 : 0;
            cb.bloomStrength = m_bloom ? m_bloomStrength : 0.0f;
            cb.vignette = m_vignette; cb.ca = m_ca; cb.grain = m_grain;
            cb.contrast = m_contrast; cb.saturation = m_saturation;
            cb.temperature = m_temperature; cb.keyValue = m_keyValue;
            cb.time = m_time; cb.gamma = 1;
            ctx.objectCB->BindCbv(c, 1, cb);
            c->IASetVertexBuffers(0, 0, nullptr);
            c->DrawInstanced(3, 1, 0, 0);
        },
        /*sideEffect*/ true);

    m_graph.Execute();
}

void PostProcessScene::OnImGui() {
    ImGui::SeparatorText("Exposure");
    ImGui::Checkbox("Auto (eye adaptation)", &m_autoExposure);
    if (m_autoExposure) {
        ImGui::SliderFloat("Key value", &m_keyValue, 0.02f, 0.5f, "%.3f");
        ImGui::SliderFloat("Adapt speed", &m_adaptSpeed, 0.1f, 6.0f, "%.1f");
    } else {
        ImGui::SliderFloat("Exposure", &m_exposure, 0.05f, 6.0f, "%.2f");
    }
    ImGui::SeparatorText("Bloom");
    ImGui::Checkbox("Enabled", &m_bloom);
    ImGui::SliderFloat("Strength", &m_bloomStrength, 0.0f, 2.0f, "%.2f");
    ImGui::SliderFloat("Threshold", &m_threshold, 0.0f, 4.0f, "%.2f");
    ImGui::SliderInt("Blur iterations", &m_blurIters, 0, 8);
    ImGui::SeparatorText("Colour grade");
    ImGui::SliderFloat("Contrast", &m_contrast, 0.5f, 1.8f, "%.2f");
    ImGui::SliderFloat("Saturation", &m_saturation, 0.0f, 2.0f, "%.2f");
    ImGui::SliderFloat("Temperature", &m_temperature, -0.3f, 0.3f, "%.2f");
    ImGui::SeparatorText("Lens / sensor");
    ImGui::SliderFloat("Vignette", &m_vignette, 0.0f, 1.0f, "%.2f");
    ImGui::SliderFloat("Chromatic aberration", &m_ca, 0.0f, 0.03f, "%.3f");
    ImGui::SliderFloat("Film grain", &m_grain, 0.0f, 0.2f, "%.3f");
}

bool PostProcessScene::PreferredCamera(float pos[3], float& yaw, float& pitch) const {
    pos[0] = 0.0f; pos[1] = 0.0f; pos[2] = -15.0f;
    yaw = 0.0f; pitch = 0.0f;
    return true;
}
