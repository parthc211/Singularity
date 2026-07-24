#include "Scenes/DeferredScene.h"

#include "Scene/Components.h"
#include "Scene/RenderSystem.h"
#include "Renderer/Renderer.h"
#include "Renderer/Mesh.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"

#include "imgui.h"
#include <cmath>
#include <cstring>
#include <random>

using namespace SGE;
using namespace DirectX;

DeferredScene::~DeferredScene() = default;

const char* DeferredScene::Description() const {
    return "Deferred rendering: a grid of cubes is drawn ONCE into a 3-target "
           "G-buffer (albedo / world-normal / world-position via MRT). A single "
           "fullscreen pass then samples those targets and accumulates every "
           "moving point light. Switch the G-buffer view to inspect each target.";
}

void DeferredScene::OnLoad(const DemoContext& ctx) {
    m_shaders.Initialize(L"Shaders");
    if (!BuildPipelines(ctx))
        return;
    BuildScene();
    BuildLights();
    m_gbuffer.Create(ctx.device, ctx.renderer->GetWidth(), ctx.renderer->GetHeight());
    m_ready = true;
}

void DeferredScene::OnUnload() {
    m_ready = false;
    m_gbuffer.Reset();
    m_geoPSO.Reset();
    m_geoRootSig.Reset();
    m_lightPSO.Reset();
    m_lightRootSig.Reset();
    m_world = World{};
    m_shaders.Shutdown();
}

bool DeferredScene::BuildPipelines(const DemoContext& ctx) {
    ID3D12Device* device = ctx.device;

    // --- Geometry pass: same b0 root CBV the forward RenderSystem feeds. ---
    D3D12_ROOT_PARAMETER geoParam      = {};
    geoParam.ParameterType             = D3D12_ROOT_PARAMETER_TYPE_CBV;
    geoParam.Descriptor.ShaderRegister = 0;
    geoParam.ShaderVisibility          = D3D12_SHADER_VISIBILITY_ALL;
    if (!m_geoRootSig.Create(device, &geoParam, 1))
        return false;

    auto gvs = m_shaders.GetOrCompile(L"GeometryPass.hlsl", "VSMain", "vs_6_0");
    auto gps = m_shaders.GetOrCompile(L"GeometryPass.hlsl", "PSMain", "ps_6_0");
    if (!gvs || !gps)
        return false;

    GraphicsPipelineDesc geoDesc;
    geoDesc.rootSignature = m_geoRootSig.Get();
    geoDesc.vs            = gvs;
    geoDesc.ps            = gps;
    geoDesc.depthEnable   = true;
    geoDesc.dsvFormat     = DXGI_FORMAT_D32_FLOAT;
    geoDesc.rtvCount      = GBuffer::kCount;
    for (uint32_t i = 0; i < GBuffer::kCount; ++i)
        geoDesc.rtvFormats[i] = GBuffer::Format(i);
    if (!m_geoPSO.Create(device, geoDesc))
        return false;

    // --- Lighting pass: SRV table (t0..t2) + light CBV (b0) + static sampler. ---
    D3D12_DESCRIPTOR_RANGE srvRange            = {};
    srvRange.RangeType                          = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    srvRange.NumDescriptors                     = GBuffer::kCount;
    srvRange.BaseShaderRegister                 = 0; // t0
    srvRange.OffsetInDescriptorsFromTableStart  = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

    D3D12_ROOT_PARAMETER lightParams[2] = {};
    lightParams[0].ParameterType                       = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    lightParams[0].DescriptorTable.NumDescriptorRanges = 1;
    lightParams[0].DescriptorTable.pDescriptorRanges   = &srvRange;
    lightParams[0].ShaderVisibility                    = D3D12_SHADER_VISIBILITY_PIXEL;
    lightParams[1].ParameterType                       = D3D12_ROOT_PARAMETER_TYPE_CBV;
    lightParams[1].Descriptor.ShaderRegister           = 0; // b0
    lightParams[1].ShaderVisibility                    = D3D12_SHADER_VISIBILITY_PIXEL;

    D3D12_STATIC_SAMPLER_DESC sampler = {};
    sampler.Filter           = D3D12_FILTER_MIN_MAG_MIP_POINT; // 1:1 G-buffer fetch
    sampler.AddressU         = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.AddressV         = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.AddressW         = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.ShaderRegister   = 0; // s0
    sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    sampler.MaxLOD           = D3D12_FLOAT32_MAX;

    if (!m_lightRootSig.Create(device, lightParams, 2, &sampler, 1))
        return false;

    auto lvs = m_shaders.GetOrCompile(L"DeferredLighting.hlsl", "VSMain", "vs_6_0");
    auto lps = m_shaders.GetOrCompile(L"DeferredLighting.hlsl", "PSMain", "ps_6_0");
    if (!lvs || !lps)
        return false;

    GraphicsPipelineDesc lightDesc;
    lightDesc.rootSignature = m_lightRootSig.Get();
    lightDesc.vs            = lvs;
    lightDesc.ps            = lps;
    lightDesc.depthEnable   = false;                       // fullscreen, no depth
    lightDesc.rtvFormat     = DXGI_FORMAT_R8G8B8A8_UNORM;  // back buffer
    return m_lightPSO.Create(device, lightDesc);
}

void DeferredScene::BuildScene() {
    m_world = World{};
    const int   n       = 7;
    const float spacing = 2.2f;
    const float offset  = (n - 1) * 0.5f * spacing;
    for (int x = 0; x < n; ++x) {
        for (int z = 0; z < n; ++z) {
            Entity e = m_world.Create();
            TransformComponent t;
            t.Position = { x * spacing - offset, 0.0f, z * spacing - offset };
            t.Scale    = { 0.8f, 0.8f, 0.8f };
            m_world.Add(e, t);
            MeshComponent mc; mc.MeshPtr = m_cube; m_world.Add(e, mc);
            MaterialComponent mat; mat.BaseColor = { 0.85f, 0.85f, 0.9f, 1.0f };
            m_world.Add(e, mat);
        }
    }
}

void DeferredScene::BuildLights() {
    m_anims.clear();
    std::mt19937 rng(1234);
    std::uniform_real_distribution<float> col(0.35f, 1.0f);
    std::uniform_real_distribution<float> place(-7.0f, 7.0f);
    std::uniform_real_distribution<float> phase(0.0f, XM_2PI);
    for (int i = 0; i < kMaxLights; ++i) {
        LightAnim a;
        a.Center      = { place(rng), 1.3f, place(rng) };
        a.OrbitRadius = 1.0f + col(rng) * 3.0f;
        a.Phase       = phase(rng);
        a.Speed       = 0.4f + col(rng) * 0.8f;
        a.Color       = { col(rng), col(rng), col(rng) };
        a.Intensity   = 2.5f;
        a.LightRadius = 5.0f;
        m_anims.push_back(a);
    }
}

void DeferredScene::OnUpdate(const DemoContext& ctx) {
    if (m_animate)
        m_time += ctx.dt;

    for (int i = 0; i < kMaxLights; ++i) {
        const LightAnim& a = m_anims[i];
        const float ang = a.Phase + m_time * a.Speed;
        m_lightData.Lights[i].Position  = { a.Center.x + cosf(ang) * a.OrbitRadius,
                                            a.Center.y,
                                            a.Center.z + sinf(ang) * a.OrbitRadius };
        m_lightData.Lights[i].Radius    = a.LightRadius;
        m_lightData.Lights[i].Color     = a.Color;
        m_lightData.Lights[i].Intensity = a.Intensity;
    }
    m_lightData.LightCount = static_cast<uint32_t>(m_lightCount);
    m_lightData.DebugMode  = static_cast<uint32_t>(m_debugMode);
    m_lightData.Ambient    = { m_ambient, m_ambient, m_ambient, 1.0f };
    m_lightData.CameraPos  = { ctx.cameraPos[0], ctx.cameraPos[1], ctx.cameraPos[2], 1.0f };
}

void DeferredScene::OnRender(const DemoContext& ctx) {
    if (!m_ready)
        return;

    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;

    // Recreate the G-buffer if the window changed size (GPU drained first).
    const uint32_t w = r->GetWidth(), h = r->GetHeight();
    if (m_gbuffer.Width() != w || m_gbuffer.Height() != h) {
        r->WaitForGPU();
        m_gbuffer.Create(ctx.device, w, h);
    }

    // ---- Geometry pass: fill the G-buffer (MRT). ----
    m_gbuffer.TransitionToRenderTargets(cmd);
    D3D12_CPU_DESCRIPTOR_HANDLE rtvs[GBuffer::kCount] = {
        m_gbuffer.Rtv(0), m_gbuffer.Rtv(1), m_gbuffer.Rtv(2)
    };
    D3D12_CPU_DESCRIPTOR_HANDLE dsv = r->GetDepthDSV();
    cmd->OMSetRenderTargets(GBuffer::kCount, rtvs, FALSE, &dsv);
    m_gbuffer.ClearRenderTargets(cmd); // depth was cleared by Renderer::BeginFrame
    cmd->SetGraphicsRootSignature(m_geoRootSig.Get());
    cmd->SetPipelineState(m_geoPSO.Get());
    // Same RenderSystem as the forward scenes — it only binds per-object CBVs and
    // draws; the bound PSO decides this writes the G-buffer instead of shading.
    ctx.renderSystem->Render(m_world, *ctx.camera, *ctx.objectCB, cmd, ctx.rootParamIndexCBV);

    // ---- Lighting pass: fullscreen, sample G-buffer, accumulate lights. ----
    m_gbuffer.TransitionToShaderResources(cmd);
    D3D12_CPU_DESCRIPTOR_HANDLE backRtv = r->GetBackBufferRTV();
    cmd->OMSetRenderTargets(1, &backRtv, FALSE, nullptr);
    cmd->SetGraphicsRootSignature(m_lightRootSig.Get());
    cmd->SetPipelineState(m_lightPSO.Get());

    ID3D12DescriptorHeap* heaps[] = { m_gbuffer.SrvHeap() };
    cmd->SetDescriptorHeaps(1, heaps);
    cmd->SetGraphicsRootDescriptorTable(0, m_gbuffer.SrvTable());

    const auto alloc = ctx.objectCB->Allocate(sizeof(LightData));
    if (alloc.Cpu) {
        std::memcpy(alloc.Cpu, &m_lightData, sizeof(LightData));
        cmd->SetGraphicsRootConstantBufferView(1, alloc.Gpu);
        cmd->IASetVertexBuffers(0, 0, nullptr); // fullscreen triangle from SV_VertexID
        cmd->DrawInstanced(3, 1, 0, 0);
    }
}

void DeferredScene::OnImGui() {
    ImGui::SliderInt("Lights", &m_lightCount, 1, kMaxLights);
    ImGui::Checkbox("Animate", &m_animate);
    ImGui::SliderFloat("Ambient", &m_ambient, 0.0f, 0.3f, "%.3f");
    const char* modes[] = { "Composite", "Albedo", "Normal", "Position" };
    ImGui::Combo("G-buffer view", &m_debugMode, modes, IM_ARRAYSIZE(modes));
    ImGui::TextDisabled("49 cubes -> 3-RT G-buffer -> %d lights", m_lightCount);
}
