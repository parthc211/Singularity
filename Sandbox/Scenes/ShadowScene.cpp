#include "Scenes/ShadowScene.h"

#include "Core/Camera.h"
#include "Renderer/Renderer.h"
#include "Renderer/Mesh.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"

#include "imgui.h"
#include <cmath>
#include <cstring>

using namespace SGE;
using namespace DirectX;

namespace {
// Must match the cbuffers in ShadowDepth.hlsl / ShadowLit.hlsl.
struct ShadowObjCB { XMFLOAT4X4 LightMVP; };
struct LitObjCB    { XMFLOAT4X4 MVP; XMFLOAT4X4 Model; XMFLOAT4X4 LightMVP; XMFLOAT4 BaseColor; };
struct LitFrameCB  { XMFLOAT4 LightDir; XMFLOAT4 CameraPos; float Bias; float TexelSize; int PcfRadius; float _pad; };

XMVECTOR ComputeLightDir(float elevationDeg, float azimuth) {
    const float el = XMConvertToRadians(elevationDeg);
    return XMVector3Normalize(XMVectorSet(cosf(el) * cosf(azimuth), -sinf(el),
                                          cosf(el) * sinf(azimuth), 0.0f));
}
}

const char* ShadowScene::Description() const {
    return "Shadow mapping: pass 1 renders scene depth from the light's view into "
           "a depth texture; pass 2 reprojects each pixel into light space and "
           "compares its depth against the map (PCF-filtered) to gate direct "
           "light. Animate the sun and tune bias / PCF radius below.";
}

XMMATRIX ShadowScene::LightViewProj() const {
    const XMVECTOR dir    = ComputeLightDir(m_lightElevation, m_lightAzimuth);
    const XMVECTOR center = XMVectorZero();
    const XMVECTOR eye    = XMVectorScale(dir, -40.0f); // back up along -dir from the scene
    const XMMATRIX view   = XMMatrixLookAtLH(eye, center, XMVectorSet(0, 1, 0, 0));
    const XMMATRIX proj   = XMMatrixOrthographicLH(60.0f, 60.0f, 0.1f, 100.0f); // covers the ground
    return view * proj;
}

void ShadowScene::BuildObjects() {
    m_objects.clear();
    auto add = [&](XMMATRIX m, XMFLOAT4 c) {
        Object o; XMStoreFloat4x4(&o.Model, m); o.Color = c; m_objects.push_back(o);
    };

    // Ground: a flattened cube.
    add(XMMatrixScaling(40.0f, 0.5f, 40.0f) * XMMatrixTranslation(0.0f, -0.25f, 0.0f),
        { 0.50f, 0.55f, 0.50f, 1.0f });

    struct Box { float x, z, sx, sy, sz; XMFLOAT4 c; };
    const Box boxes[] = {
        { -6, -4, 2.0f, 3.0f, 2.0f, { 0.80f, 0.30f, 0.30f, 1 } },
        {  5, -6, 1.5f, 5.0f, 1.5f, { 0.30f, 0.60f, 0.85f, 1 } },
        {  8,  4, 2.5f, 1.5f, 2.5f, { 0.85f, 0.70f, 0.30f, 1 } },
        { -7,  6, 1.2f, 4.0f, 1.2f, { 0.50f, 0.80f, 0.40f, 1 } },
        {  0,  0, 1.5f, 2.0f, 1.5f, { 0.85f, 0.85f, 0.90f, 1 } },
        { -2,  9, 2.0f, 1.0f, 3.0f, { 0.70f, 0.40f, 0.70f, 1 } },
        {  3,  9, 1.0f, 2.5f, 1.0f, { 0.40f, 0.75f, 0.70f, 1 } },
        { 10, -2, 1.5f, 3.5f, 1.5f, { 0.85f, 0.50f, 0.30f, 1 } },
    };
    for (const Box& b : boxes)
        add(XMMatrixScaling(b.sx, b.sy, b.sz) * XMMatrixTranslation(b.x, b.sy * 0.5f, b.z), b.c);
}

bool ShadowScene::BuildPipelines(const DemoContext& ctx) {
    ID3D12Device* device = ctx.device;
    m_shaders.Initialize(L"Shaders");

    // --- Shadow pass: one CBV (the light MVP), vertex-only, depth-only PSO. ---
    D3D12_ROOT_PARAMETER sp      = {};
    sp.ParameterType             = D3D12_ROOT_PARAMETER_TYPE_CBV;
    sp.Descriptor.ShaderRegister = 0;
    sp.ShaderVisibility          = D3D12_SHADER_VISIBILITY_VERTEX;
    if (!m_shadowRootSig.Create(device, &sp, 1))
        return false;

    auto svs = m_shaders.GetOrCompile(L"ShadowDepth.hlsl", "VSMain", "vs_6_0");
    if (!svs) return false;
    GraphicsPipelineDesc sd;
    sd.rootSignature = m_shadowRootSig.Get();
    sd.vs            = svs;
    sd.ps            = nullptr;                 // depth only
    sd.rtvCount      = 0;                       // no colour targets
    sd.depthEnable   = true;
    sd.dsvFormat     = DXGI_FORMAT_D32_FLOAT;
    if (!m_shadowPSO.Create(device, sd))
        return false;

    // --- Main pass: b0 obj CBV, b1 frame CBV, SRV table (shadow map), comparison sampler. ---
    D3D12_DESCRIPTOR_RANGE range              = {};
    range.RangeType                            = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    range.NumDescriptors                       = 1;
    range.BaseShaderRegister                   = 0; // t0
    range.OffsetInDescriptorsFromTableStart    = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

    D3D12_ROOT_PARAMETER lp[3] = {};
    lp[0].ParameterType             = D3D12_ROOT_PARAMETER_TYPE_CBV;
    lp[0].Descriptor.ShaderRegister = 0; // b0
    lp[0].ShaderVisibility          = D3D12_SHADER_VISIBILITY_ALL;
    lp[1].ParameterType             = D3D12_ROOT_PARAMETER_TYPE_CBV;
    lp[1].Descriptor.ShaderRegister = 1; // b1
    lp[1].ShaderVisibility          = D3D12_SHADER_VISIBILITY_ALL;
    lp[2].ParameterType                       = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    lp[2].DescriptorTable.NumDescriptorRanges = 1;
    lp[2].DescriptorTable.pDescriptorRanges   = &range;
    lp[2].ShaderVisibility                    = D3D12_SHADER_VISIBILITY_PIXEL;

    D3D12_STATIC_SAMPLER_DESC samp = {};
    samp.Filter           = D3D12_FILTER_COMPARISON_MIN_MAG_LINEAR_MIP_POINT; // hardware PCF
    samp.AddressU         = D3D12_TEXTURE_ADDRESS_MODE_BORDER;
    samp.AddressV         = D3D12_TEXTURE_ADDRESS_MODE_BORDER;
    samp.AddressW         = D3D12_TEXTURE_ADDRESS_MODE_BORDER;
    samp.BorderColor      = D3D12_STATIC_BORDER_COLOR_OPAQUE_WHITE; // outside light frustum = lit
    samp.ComparisonFunc   = D3D12_COMPARISON_FUNC_LESS_EQUAL;
    samp.ShaderRegister   = 0; // s0
    samp.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    samp.MaxLOD           = D3D12_FLOAT32_MAX;
    if (!m_litRootSig.Create(device, lp, 3, &samp, 1))
        return false;

    auto lvs = m_shaders.GetOrCompile(L"ShadowLit.hlsl", "VSMain", "vs_6_0");
    auto lps = m_shaders.GetOrCompile(L"ShadowLit.hlsl", "PSMain", "ps_6_0");
    if (!lvs || !lps) return false;
    GraphicsPipelineDesc ld;
    ld.rootSignature = m_litRootSig.Get();
    ld.vs = lvs; ld.ps = lps;
    ld.depthEnable = true;
    ld.rtvFormat   = DXGI_FORMAT_R8G8B8A8_UNORM;
    return m_litPSO.Create(device, ld);
}

void ShadowScene::OnLoad(const DemoContext& ctx) {
    BuildObjects();
    const bool sm = m_shadowMap.Create(ctx.device, kShadowSize);
    const bool pl = BuildPipelines(ctx);
    m_ready = sm && pl;
}

void ShadowScene::OnUnload() {
    m_shadowMap.Reset();
    m_shadowPSO.Reset();
    m_litPSO.Reset();
    m_shadowRootSig.Reset();
    m_litRootSig.Reset();
    m_shaders.Shutdown();
    m_objects.clear();
    m_ready = false;
}

void ShadowScene::OnUpdate(const DemoContext& ctx) {
    if (m_animateLight) {
        m_time += ctx.dt;
        m_lightAzimuth = 0.6f + m_time * 0.25f;
    }
}

void ShadowScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;
    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;

    const XMMATRIX camVP   = ctx.camera->GetViewProjection();
    const XMMATRIX lightVP = LightViewProj();
    const XMVECTOR lightDir = ComputeLightDir(m_lightElevation, m_lightAzimuth);

    // ---- Pass 1: scene depth from the light's POV into the shadow map. ----
    m_shadowMap.BeginShadowPass(cmd);
    cmd->SetGraphicsRootSignature(m_shadowRootSig.Get());
    cmd->SetPipelineState(m_shadowPSO.Get());
    cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    for (const Object& o : m_objects) {
        ShadowObjCB cb;
        XMStoreFloat4x4(&cb.LightMVP, XMLoadFloat4x4(&o.Model) * lightVP);
        const auto a = ctx.objectCB->Allocate(sizeof(cb));
        if (!a.Cpu) continue;
        std::memcpy(a.Cpu, &cb, sizeof(cb));
        cmd->SetGraphicsRootConstantBufferView(0, a.Gpu);
        m_cube->Draw(cmd);
    }
    m_shadowMap.EndShadowPass(cmd);

    // ---- Restore the back buffer + depth + full viewport for the main pass. ----
    D3D12_CPU_DESCRIPTOR_HANDLE backRtv = r->GetBackBufferRTV();
    D3D12_CPU_DESCRIPTOR_HANDLE depthDsv = r->GetDepthDSV();
    cmd->OMSetRenderTargets(1, &backRtv, FALSE, &depthDsv);
    D3D12_VIEWPORT vp = { 0, 0, float(r->GetWidth()), float(r->GetHeight()), 0.0f, 1.0f };
    D3D12_RECT     sc = { 0, 0, LONG(r->GetWidth()), LONG(r->GetHeight()) };
    cmd->RSSetViewports(1, &vp);
    cmd->RSSetScissorRects(1, &sc);

    // ---- Pass 2: shade + sample the shadow map. ----
    cmd->SetGraphicsRootSignature(m_litRootSig.Get());
    cmd->SetPipelineState(m_litPSO.Get());
    ID3D12DescriptorHeap* heaps[] = { m_shadowMap.SrvHeap() };
    cmd->SetDescriptorHeaps(1, heaps);
    cmd->SetGraphicsRootDescriptorTable(2, m_shadowMap.Srv());

    LitFrameCB fcb;
    XMStoreFloat4(&fcb.LightDir, lightDir);
    fcb.CameraPos = { ctx.cameraPos[0], ctx.cameraPos[1], ctx.cameraPos[2], 1.0f };
    fcb.Bias      = m_bias;
    fcb.TexelSize = 1.0f / static_cast<float>(kShadowSize);
    fcb.PcfRadius = m_pcfRadius;
    fcb._pad      = 0.0f;
    const auto fa = ctx.objectCB->Allocate(sizeof(fcb));
    if (fa.Cpu) {
        std::memcpy(fa.Cpu, &fcb, sizeof(fcb));
        cmd->SetGraphicsRootConstantBufferView(1, fa.Gpu);
    }

    for (const Object& o : m_objects) {
        LitObjCB cb;
        XMStoreFloat4x4(&cb.MVP,      XMLoadFloat4x4(&o.Model) * camVP);
        cb.Model = o.Model;
        XMStoreFloat4x4(&cb.LightMVP, XMLoadFloat4x4(&o.Model) * lightVP);
        cb.BaseColor = o.Color;
        const auto a = ctx.objectCB->Allocate(sizeof(cb));
        if (!a.Cpu) continue;
        std::memcpy(a.Cpu, &cb, sizeof(cb));
        cmd->SetGraphicsRootConstantBufferView(0, a.Gpu);
        m_cube->Draw(cmd);
    }
}

void ShadowScene::OnImGui() {
    ImGui::Checkbox("Animate sun", &m_animateLight);
    ImGui::SliderFloat("Light elevation", &m_lightElevation, 20.0f, 80.0f, "%.0f deg");
    ImGui::SliderFloat("Light azimuth",   &m_lightAzimuth, 0.0f, 6.28f, "%.2f");
    ImGui::SliderFloat("Depth bias",      &m_bias, 0.0f, 0.01f, "%.4f");
    ImGui::SliderInt("PCF radius",        &m_pcfRadius, 0, 4);
    ImGui::TextDisabled("%dx%d shadow map, %d PCF taps",
                        kShadowSize, kShadowSize, (2 * m_pcfRadius + 1) * (2 * m_pcfRadius + 1));
}
