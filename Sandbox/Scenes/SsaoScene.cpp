#include "Scenes/SsaoScene.h"

#include "Core/Camera.h"
#include "Renderer/Renderer.h"
#include "Renderer/Mesh.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"
#include "Scene/Components.h"
#include "Scene/RenderSystem.h"

#include "imgui.h"
#include <DirectXMath.h>
#include <random>
#include <cmath>
#include <cstring>

using namespace SGE;
using namespace DirectX;

namespace {
struct SsaoCB {
    XMFLOAT4X4 ViewProj;
    XMFLOAT4   CameraPos;
    float      Radius, Bias, Power, Strength;
    XMFLOAT2   Screen; int SampleCount; float _pad;
    XMFLOAT4   Kernel[32];
};
struct BlurCB      { XMFLOAT2 TexelSize; XMFLOAT2 _pad; };
struct CompositeCB { int Mode; float Ambient; XMFLOAT2 _pad; };

void CreateTex2DSrv(ID3D12Device* device, ID3D12Resource* res, DXGI_FORMAT fmt,
                    D3D12_CPU_DESCRIPTOR_HANDLE dst) {
    D3D12_SHADER_RESOURCE_VIEW_DESC s = {};
    s.Format                  = fmt;
    s.ViewDimension           = D3D12_SRV_DIMENSION_TEXTURE2D;
    s.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    s.Texture2D.MipLevels     = 1;
    device->CreateShaderResourceView(res, &s, dst);
}
}

const char* SsaoScene::Description() const {
    return "Screen-space ambient occlusion on the deferred G-buffer. For each "
           "pixel a hemisphere of samples (oriented to the normal) is projected "
           "and tested against stored depth to estimate how occluded it is; the "
           "result is blurred and modulates albedo. Switch the view to see raw AO.";
}

void SsaoScene::BuildScene() {
    m_world = World{};
    auto add = [&](XMFLOAT3 pos, XMFLOAT3 scale, XMFLOAT4 col) {
        Entity e = m_world.Create();
        TransformComponent t; t.Position = pos; t.Scale = scale;
        m_world.Add(e, t);
        MeshComponent mc; mc.MeshPtr = m_cube; m_world.Add(e, mc);
        MaterialComponent mat; mat.BaseColor = col; m_world.Add(e, mat);
    };

    add({ 0, -0.5f, 0 }, { 30, 1, 30 }, { 0.8f, 0.8f, 0.8f, 1 }); // ground

    // Clustered / stacked / touching boxes — creases and contacts where AO shows.
    add({ -3, 1.0f, -2 }, { 2, 2, 2 }, { 0.85f, 0.85f, 0.9f, 1 });
    add({ -1.2f, 0.6f, -2.4f }, { 1.2f, 1.2f, 1.2f }, { 0.8f, 0.8f, 0.85f, 1 });
    add({ 3, 1.5f, 1 }, { 1.4f, 3.0f, 1.4f }, { 0.9f, 0.9f, 0.9f, 1 });
    add({ 3, 4.0f, 1 }, { 1.0f, 1.0f, 1.0f }, { 0.85f, 0.85f, 0.9f, 1 }); // stacked
    add({ 0, 0.75f, 3 }, { 4, 1.5f, 1 }, { 0.82f, 0.82f, 0.88f, 1 });
    add({ -4, 0.75f, 3 }, { 1.5f, 1.5f, 1.5f }, { 0.88f, 0.88f, 0.9f, 1 });
    add({ 1.0f, 0.5f, -4 }, { 1, 1, 1 }, { 0.8f, 0.85f, 0.85f, 1 });
    add({ 2.2f, 0.5f, -4 }, { 1, 1, 1 }, { 0.85f, 0.8f, 0.8f, 1 });
    add({ -3, 0.5f, 4 }, { 1, 1, 1 }, { 0.85f, 0.85f, 0.85f, 1 });
}

void SsaoScene::BuildKernel() {
    m_kernel.clear();
    std::mt19937 rng(1234);
    std::uniform_real_distribution<float> u01(0.0f, 1.0f), u11(-1.0f, 1.0f);
    for (int i = 0; i < kKernelSize; ++i) {
        XMVECTOR v = XMVector3Normalize(XMVectorSet(u11(rng), u11(rng), u01(rng), 0.0f)); // hemisphere z+
        float t = (float)i / kKernelSize;
        float scale = 0.1f + 0.9f * (t * t);   // bias samples toward the centre
        v = XMVectorScale(v, u01(rng) * scale);
        XMFLOAT4 k; XMStoreFloat4(&k, v); k.w = 0.0f;
        m_kernel.push_back(k);
    }
}

bool SsaoScene::BuildPipelines(const DemoContext& ctx) {
    ID3D12Device* device = ctx.device;
    m_shaders.Initialize(L"Shaders");

    // Geometry pass: same b0 per-object CBV the deferred RenderSystem feeds.
    D3D12_ROOT_PARAMETER geoP      = {};
    geoP.ParameterType             = D3D12_ROOT_PARAMETER_TYPE_CBV;
    geoP.Descriptor.ShaderRegister = 0;
    geoP.ShaderVisibility          = D3D12_SHADER_VISIBILITY_ALL;
    if (!m_geoRootSig.Create(device, &geoP, 1)) return false;

    D3D12_STATIC_SAMPLER_DESC samp = {};
    samp.Filter           = D3D12_FILTER_MIN_MAG_MIP_POINT; // exact G-buffer/AO fetches
    samp.AddressU = samp.AddressV = samp.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    samp.ShaderRegister   = 0;
    samp.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    samp.MaxLOD           = D3D12_FLOAT32_MAX;

    auto makePostRootSig = [&](RootSignature& rs, UINT srvCount) {
        D3D12_DESCRIPTOR_RANGE range = {};
        range.RangeType                         = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
        range.NumDescriptors                    = srvCount;
        range.BaseShaderRegister                = 0;
        range.OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;
        D3D12_ROOT_PARAMETER p[2] = {};
        p[0].ParameterType                       = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
        p[0].DescriptorTable.NumDescriptorRanges = 1;
        p[0].DescriptorTable.pDescriptorRanges   = &range;
        p[0].ShaderVisibility                    = D3D12_SHADER_VISIBILITY_PIXEL;
        p[1].ParameterType                       = D3D12_ROOT_PARAMETER_TYPE_CBV;
        p[1].Descriptor.ShaderRegister           = 0;
        p[1].ShaderVisibility                    = D3D12_SHADER_VISIBILITY_PIXEL;
        return rs.Create(device, p, 2, &samp, 1);
    };
    if (!makePostRootSig(m_post1RootSig, 1)) return false; // blur
    if (!makePostRootSig(m_post2RootSig, 2)) return false; // ssao, composite

    auto gvs = m_shaders.GetOrCompile(L"GeometryPass.hlsl", "VSMain", "vs_6_0");
    auto gps = m_shaders.GetOrCompile(L"GeometryPass.hlsl", "PSMain", "ps_6_0");
    auto svs = m_shaders.GetOrCompile(L"Ssao.hlsl", "VSMain", "vs_6_0");
    auto sps = m_shaders.GetOrCompile(L"Ssao.hlsl", "PSMain", "ps_6_0");
    auto bvs = m_shaders.GetOrCompile(L"SsaoBlur.hlsl", "VSMain", "vs_6_0");
    auto bps = m_shaders.GetOrCompile(L"SsaoBlur.hlsl", "PSMain", "ps_6_0");
    auto cvs = m_shaders.GetOrCompile(L"SsaoComposite.hlsl", "VSMain", "vs_6_0");
    auto cps = m_shaders.GetOrCompile(L"SsaoComposite.hlsl", "PSMain", "ps_6_0");
    if (!gvs || !gps || !svs || !sps || !bvs || !bps || !cvs || !cps) return false;

    GraphicsPipelineDesc geo;
    geo.rootSignature = m_geoRootSig.Get(); geo.vs = gvs; geo.ps = gps;
    geo.depthEnable = true; geo.rtvCount = GBuffer::kCount;
    for (uint32_t i = 0; i < GBuffer::kCount; ++i) geo.rtvFormats[i] = GBuffer::Format(i);
    if (!m_geoPSO.Create(device, geo)) return false;

    GraphicsPipelineDesc ssao;
    ssao.rootSignature = m_post2RootSig.Get(); ssao.vs = svs; ssao.ps = sps;
    ssao.depthEnable = false; ssao.rtvFormat = DXGI_FORMAT_R8_UNORM;
    if (!m_ssaoPSO.Create(device, ssao)) return false;

    GraphicsPipelineDesc blur;
    blur.rootSignature = m_post1RootSig.Get(); blur.vs = bvs; blur.ps = bps;
    blur.depthEnable = false; blur.rtvFormat = DXGI_FORMAT_R8_UNORM;
    if (!m_blurPSO.Create(device, blur)) return false;

    GraphicsPipelineDesc comp;
    comp.rootSignature = m_post2RootSig.Get(); comp.vs = cvs; comp.ps = cps;
    comp.depthEnable = false; comp.rtvFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
    return m_compositePSO.Create(device, comp);
}

bool SsaoScene::CreateTargets(ID3D12Device* device, uint32_t w, uint32_t h) {
    m_targetW = w; m_targetH = h;
    if (!m_srvHeap) {
        D3D12_DESCRIPTOR_HEAP_DESC d = {};
        d.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
        d.NumDescriptors = 5;
        d.Flags          = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        if (FAILED(device->CreateDescriptorHeap(&d, IID_PPV_ARGS(&m_srvHeap)))) return false;
        m_srvStride = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
    }
    if (!m_gbuffer.Create(device, w, h)) return false;
    const float white[4] = { 1, 1, 1, 1 };
    if (!m_ao.Create(device, w, h, DXGI_FORMAT_R8_UNORM, white)) return false;
    if (!m_aoBlur.Create(device, w, h, DXGI_FORMAT_R8_UNORM, white)) return false;

    // Slots: 0=normal, 1=position, 2=AO, 3=albedo, 4=AOblurred.
    CreateTex2DSrv(device, m_gbuffer.Resource(GBuffer::Normal),   GBuffer::Format(GBuffer::Normal),   SlotCpu(0));
    CreateTex2DSrv(device, m_gbuffer.Resource(GBuffer::Position), GBuffer::Format(GBuffer::Position), SlotCpu(1));
    m_ao.CreateSrvInto(device, SlotCpu(2));
    CreateTex2DSrv(device, m_gbuffer.Resource(GBuffer::Albedo),   GBuffer::Format(GBuffer::Albedo),   SlotCpu(3));
    m_aoBlur.CreateSrvInto(device, SlotCpu(4));
    return true;
}

D3D12_GPU_DESCRIPTOR_HANDLE SsaoScene::SlotGpu(uint32_t slot) const {
    D3D12_GPU_DESCRIPTOR_HANDLE h = m_srvHeap->GetGPUDescriptorHandleForHeapStart();
    h.ptr += static_cast<UINT64>(slot) * m_srvStride; return h;
}
D3D12_CPU_DESCRIPTOR_HANDLE SsaoScene::SlotCpu(uint32_t slot) const {
    D3D12_CPU_DESCRIPTOR_HANDLE h = m_srvHeap->GetCPUDescriptorHandleForHeapStart();
    h.ptr += static_cast<SIZE_T>(slot) * m_srvStride; return h;
}

void SsaoScene::OnLoad(const DemoContext& ctx) {
    BuildScene();
    BuildKernel();
    const bool pl = BuildPipelines(ctx);
    const bool tg = CreateTargets(ctx.device, ctx.renderer->GetWidth(), ctx.renderer->GetHeight());
    m_ready = pl && tg;
}

void SsaoScene::OnUnload() {
    m_gbuffer.Reset(); m_ao.Reset(); m_aoBlur.Reset();
    m_srvHeap.Reset(); m_srvStride = 0; m_targetW = m_targetH = 0;
    m_geoPSO.Reset(); m_ssaoPSO.Reset(); m_blurPSO.Reset(); m_compositePSO.Reset();
    m_geoRootSig.Reset(); m_post1RootSig.Reset(); m_post2RootSig.Reset();
    m_shaders.Shutdown();
    m_world = World{};
    m_ready = false;
}

void SsaoScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;
    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;

    const uint32_t w = r->GetWidth(), h = r->GetHeight();
    if (w != m_targetW || h != m_targetH) { r->WaitForGPU(); CreateTargets(ctx.device, w, h); }

    // ---- Geometry -> G-buffer (reusing the deferred geometry path) ----
    m_gbuffer.TransitionToRenderTargets(cmd);
    D3D12_CPU_DESCRIPTOR_HANDLE rtvs[GBuffer::kCount] = { m_gbuffer.Rtv(0), m_gbuffer.Rtv(1), m_gbuffer.Rtv(2) };
    D3D12_CPU_DESCRIPTOR_HANDLE dsv = r->GetDepthDSV();
    cmd->OMSetRenderTargets(GBuffer::kCount, rtvs, FALSE, &dsv);
    m_gbuffer.ClearRenderTargets(cmd);
    cmd->SetGraphicsRootSignature(m_geoRootSig.Get());
    cmd->SetPipelineState(m_geoPSO.Get());
    cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ctx.renderSystem->Render(m_world, *ctx.camera, *ctx.objectCB, cmd, ctx.rootParamIndexCBV);

    m_gbuffer.TransitionToShaderResources(cmd);
    ID3D12DescriptorHeap* heaps[] = { m_srvHeap.Get() };
    cmd->SetDescriptorHeaps(1, heaps);
    cmd->IASetVertexBuffers(0, 0, nullptr);

    auto fullscreen = [&] { cmd->DrawInstanced(3, 1, 0, 0); };

    // ---- SSAO -> m_ao ----
    m_ao.TransitionTo(cmd, D3D12_RESOURCE_STATE_RENDER_TARGET);
    D3D12_CPU_DESCRIPTOR_HANDLE aoRtv = m_ao.Rtv();
    cmd->OMSetRenderTargets(1, &aoRtv, FALSE, nullptr);
    cmd->SetGraphicsRootSignature(m_post2RootSig.Get());
    cmd->SetPipelineState(m_ssaoPSO.Get());
    cmd->SetGraphicsRootDescriptorTable(0, SlotGpu(0)); // normal, position
    {
        SsaoCB cb{};
        XMStoreFloat4x4(&cb.ViewProj, ctx.camera->GetViewProjection());
        cb.CameraPos = { ctx.cameraPos[0], ctx.cameraPos[1], ctx.cameraPos[2], 1.0f };
        cb.Radius = m_radius; cb.Bias = m_bias; cb.Power = m_power; cb.Strength = m_strength;
        cb.Screen = { float(w), float(h) }; cb.SampleCount = m_samples; cb._pad = 0.0f;
        std::memcpy(cb.Kernel, m_kernel.data(), sizeof(cb.Kernel));
        auto a = ctx.objectCB->Allocate(sizeof(cb));
        if (a.Cpu) { std::memcpy(a.Cpu, &cb, sizeof(cb)); cmd->SetGraphicsRootConstantBufferView(1, a.Gpu); }
    }
    fullscreen();

    // ---- Blur -> m_aoBlur ----
    m_ao.TransitionTo(cmd, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    m_aoBlur.TransitionTo(cmd, D3D12_RESOURCE_STATE_RENDER_TARGET);
    D3D12_CPU_DESCRIPTOR_HANDLE blurRtv = m_aoBlur.Rtv();
    cmd->OMSetRenderTargets(1, &blurRtv, FALSE, nullptr);
    cmd->SetGraphicsRootSignature(m_post1RootSig.Get());
    cmd->SetPipelineState(m_blurPSO.Get());
    cmd->SetGraphicsRootDescriptorTable(0, SlotGpu(2)); // AO
    {
        BlurCB cb{ { 1.0f / w, 1.0f / h }, { 0, 0 } };
        auto a = ctx.objectCB->Allocate(sizeof(cb));
        if (a.Cpu) { std::memcpy(a.Cpu, &cb, sizeof(cb)); cmd->SetGraphicsRootConstantBufferView(1, a.Gpu); }
    }
    fullscreen();

    // ---- Composite -> back buffer ----
    m_aoBlur.TransitionTo(cmd, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    D3D12_CPU_DESCRIPTOR_HANDLE backRtv = r->GetBackBufferRTV();
    cmd->OMSetRenderTargets(1, &backRtv, FALSE, nullptr);
    cmd->SetGraphicsRootSignature(m_post2RootSig.Get());
    cmd->SetPipelineState(m_compositePSO.Get());
    cmd->SetGraphicsRootDescriptorTable(0, SlotGpu(3)); // albedo, AOblurred
    {
        CompositeCB cb{ m_mode, m_ambient, { 0, 0 } };
        auto a = ctx.objectCB->Allocate(sizeof(cb));
        if (a.Cpu) { std::memcpy(a.Cpu, &cb, sizeof(cb)); cmd->SetGraphicsRootConstantBufferView(1, a.Gpu); }
    }
    fullscreen();
}

void SsaoScene::OnImGui() {
    const char* modes[] = { "Albedo x AO", "AO only", "Albedo only (no AO)" };
    ImGui::Combo("View", &m_mode, modes, IM_ARRAYSIZE(modes));
    ImGui::SliderFloat("Radius",   &m_radius, 0.1f, 4.0f, "%.2f");
    ImGui::SliderFloat("Bias",     &m_bias, 0.0f, 0.2f, "%.3f");
    ImGui::SliderFloat("Power",    &m_power, 0.5f, 4.0f, "%.2f");
    ImGui::SliderFloat("Strength", &m_strength, 0.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("Ambient floor", &m_ambient, 0.0f, 0.6f, "%.2f");
    ImGui::SliderInt("Samples", &m_samples, 4, 32);
}
