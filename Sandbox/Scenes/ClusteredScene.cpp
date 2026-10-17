#include "Scenes/ClusteredScene.h"

#include "Renderer/Renderer.h"
#include "Renderer/Mesh.h"
#include "Renderer/DX12/RootSignatureBuilder.h"
#include "Core/Camera.h"

#include "imgui.h"
#include <DirectXMath.h>
#include <cmath>
#include <cstring>
#include <random>

using namespace SGE;
using namespace DirectX;

namespace {
constexpr float kNear = 0.1f, kFar = 100.0f;

struct AabbCB  { XMFLOAT4X4 invProj; uint32_t grid[3]; float znear; float zfar; float screen[2]; float pad; };
struct CullCB  { XMFLOAT4X4 view; uint32_t grid[3]; uint32_t lightCount; uint32_t maxIndices; uint32_t pad[3]; };
struct ObjCB   { XMFLOAT4X4 mvp; XMFLOAT4X4 model; XMFLOAT4 albedo; };
struct FrameCB {
    XMFLOAT4X4 view; uint32_t grid[3]; uint32_t mode;
    float znear; float zfar; float screen[2];
    float camPos[3]; float tileX; float tileY; float pad[3];
};
constexpr uint32_t Groups(uint32_t n) { return (n + 63) / 64; }
} // namespace

const char* ClusteredScene::Description() const {
    return "Clustered forward shading. A compute pass builds a view-frustum grid "
           "of froxel AABBs; a second compute pass culls hundreds of moving point "
           "lights into per-froxel index lists (atomic-appended); the forward pass "
           "shades each pixel looping only its froxel's lights. Cost scales with "
           "local light density, not total count. Switch to the heatmap view to "
           "see how many lights each froxel ended up with.";
}

void ClusteredScene::OnLoad(const DemoContext& ctx) {
    m_shaders.Initialize(L"Shaders");
    m_upload.Init(ctx.device, 512 * 1024);
    if (!BuildPipelines(ctx)) return;
    BuildLights();
    if (!CreateBuffers(ctx, ctx.renderer->GetWidth(), ctx.renderer->GetHeight())) return;
    m_ready = true;
}

void ClusteredScene::OnUnload() {
    m_ready = false;
    m_aabb.Reset(); m_lightGrid.Reset(); m_indexList.Reset(); m_counter.Reset();
    m_aabbPSO.Reset(); m_cullPSO.Reset(); m_resetPSO.Reset(); m_shadePSO.Reset();
    m_aabbRS.Reset(); m_cullRS.Reset(); m_shadeRS.Reset();
    m_upload.Shutdown();
    m_shaders.Shutdown();
}

bool ClusteredScene::BuildPipelines(const DemoContext& ctx) {
    ID3D12Device* dev = ctx.device;
    const auto NONE = D3D12_ROOT_SIGNATURE_FLAG_NONE;

    // AABB build: cbuffer + aabb UAV.
    if (!RootSignatureBuilder().Cbv(0).Uav(0).Build(dev, m_aabbRS, NONE)) return false;
    auto aabbCS = m_shaders.GetOrCompile(L"ClusterAABB.hlsl", "CSMain", "cs_6_0");
    if (!aabbCS || !m_aabbPSO.Create(dev, m_aabbRS.Get(), aabbCS)) return false;

    // Cull: cbuffer + lights/aabb SRV + grid/indexlist/counter UAV.
    if (!RootSignatureBuilder().Cbv(0).Srv(0).Srv(1).Uav(0).Uav(1).Uav(2)
             .Build(dev, m_cullRS, NONE)) return false;
    auto cullCS  = m_shaders.GetOrCompile(L"ClusterCull.hlsl", "CSMain",  "cs_6_0");
    auto resetCS = m_shaders.GetOrCompile(L"ClusterCull.hlsl", "CSReset", "cs_6_0");
    if (!cullCS || !resetCS) return false;
    if (!m_cullPSO.Create(dev, m_cullRS.Get(), cullCS)) return false;
    if (!m_resetPSO.Create(dev, m_cullRS.Get(), resetCS)) return false;

    // Shade: per-object CBV + frame CBV + lights/grid/indexlist SRV.
    if (!RootSignatureBuilder().Cbv(0).Cbv(1).Srv(0).Srv(1).Srv(2).Build(dev, m_shadeRS))
        return false;
    auto vs = m_shaders.GetOrCompile(L"ClusterShade.hlsl", "VSMain", "vs_6_0");
    auto ps = m_shaders.GetOrCompile(L"ClusterShade.hlsl", "PSMain", "ps_6_0");
    if (!vs || !ps) return false;
    GraphicsPipelineDesc d; d.rootSignature = m_shadeRS.Get(); d.vs = vs; d.ps = ps;
    d.depthEnable = true; d.dsvFormat = DXGI_FORMAT_D32_FLOAT;
    return m_shadePSO.Create(dev, d);
}

void ClusteredScene::BuildLights() {
    m_anims.clear();
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> place(-18.0f, 18.0f), y(0.4f, 4.0f),
        phase(0.0f, XM_2PI), spd(0.3f, 1.2f), orb(0.5f, 2.5f);
    for (int i = 0; i < 2048; ++i)
        m_anims.push_back({ { place(rng), y(rng), place(rng) }, orb(rng), phase(rng), spd(rng) });
    m_lights.resize(2048);
}

bool ClusteredScene::CreateBuffers(const DemoContext& ctx, uint32_t w, uint32_t h) {
    m_w = w; m_h = h;
    m_tilesX = (w + kTileSize - 1) / kTileSize;
    m_tilesY = (h + kTileSize - 1) / kTileSize;
    m_froxels = m_tilesX * m_tilesY * kSlices;
    m_maxIndices = m_froxels * kMaxPerFroxel;

    if (!m_aabb.Create(ctx.device, uint64_t(m_froxels) * 32, true)) return false;
    if (!m_lightGrid.Create(ctx.device, uint64_t(m_froxels) * 8, true)) return false;
    if (!m_indexList.Create(ctx.device, uint64_t(m_maxIndices) * 4, true)) return false;
    if (!m_counter.Create(ctx.device, 256, true)) return false;
    m_clustersDirty = true;
    return true;
}

void ClusteredScene::OnUpdate(const DemoContext& ctx) {
    if (m_animate) m_time += ctx.dt;
    const int n = m_lightCount;
    std::mt19937 rng(99);
    std::uniform_real_distribution<float> col(0.3f, 1.0f);
    for (int i = 0; i < n; ++i) {
        const LightAnim& a = m_anims[i];
        const float ang = a.phase + m_time * a.speed;
        m_lights[i].pos[0] = a.base[0] + cosf(ang) * a.orbit;
        m_lights[i].pos[1] = a.base[1];
        m_lights[i].pos[2] = a.base[2] + sinf(ang) * a.orbit;
        m_lights[i].radius = 3.2f;
        // Deterministic vivid colour from the index.
        m_lights[i].color[0] = 0.5f + 0.5f * sinf(float(i) * 0.9f);
        m_lights[i].color[1] = 0.5f + 0.5f * sinf(float(i) * 0.9f + 2.1f);
        m_lights[i].color[2] = 0.5f + 0.5f * sinf(float(i) * 0.9f + 4.2f);
        m_lights[i].intensity = 2.2f;
    }
}

void ClusteredScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;
    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;
    auto& prof = r->GetProfiler();

    const uint32_t w = r->GetWidth(), h = r->GetHeight();
    if (w != m_w || h != m_h) { r->WaitForGPU(); CreateBuffers(ctx, w, h); }

    m_upload.BeginFrame(r->GetFrameIndex());
    auto push = [&](const void* d, size_t n) {
        auto a = m_upload.Allocate(n); std::memcpy(a.Cpu, d, n); return a.Gpu;
    };

    const XMMATRIX view     = ctx.camera->GetView();
    const XMMATRIX viewProj = ctx.camera->GetViewProjection();
    const XMMATRIX invProj  = XMMatrixInverse(nullptr, ctx.camera->GetProjection());
    const uint32_t grid[3]  = { m_tilesX, m_tilesY, kSlices };

    // Upload this frame's lights.
    const D3D12_GPU_VIRTUAL_ADDRESS lightsVA =
        push(m_lights.data(), size_t(m_lightCount) * sizeof(Light));

    // ---- 1. Froxel AABBs (only when projection/resolution changed) ----
    if (m_clustersDirty) {
        const uint32_t id = prof.BeginRegion(cmd, "ClusterAABB");
        AabbCB cb = {};
        XMStoreFloat4x4(&cb.invProj, invProj);
        cb.grid[0] = grid[0]; cb.grid[1] = grid[1]; cb.grid[2] = grid[2];
        cb.znear = kNear; cb.zfar = kFar; cb.screen[0] = float(w); cb.screen[1] = float(h);
        m_aabb.TransitionTo(cmd, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        cmd->SetComputeRootSignature(m_aabbRS.Get());
        cmd->SetPipelineState(m_aabbPSO.Get());
        cmd->SetComputeRootConstantBufferView(0, push(&cb, sizeof(cb)));
        cmd->SetComputeRootUnorderedAccessView(1, m_aabb.Gpu());
        cmd->Dispatch(Groups(m_froxels), 1, 1);
        m_aabb.TransitionTo(cmd, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        prof.EndRegion(cmd, id);
        m_clustersDirty = false;
    }

    // ---- 2. Light culling ----
    {
        const uint32_t id = prof.BeginRegion(cmd, "ClusterCull");
        CullCB cb = {};
        XMStoreFloat4x4(&cb.view, view);
        cb.grid[0] = grid[0]; cb.grid[1] = grid[1]; cb.grid[2] = grid[2];
        cb.lightCount = uint32_t(m_lightCount); cb.maxIndices = m_maxIndices;
        const D3D12_GPU_VIRTUAL_ADDRESS cullCbVA = push(&cb, sizeof(cb));

        m_counter.TransitionTo(cmd, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        cmd->SetComputeRootSignature(m_cullRS.Get());
        cmd->SetPipelineState(m_resetPSO.Get());
        cmd->SetComputeRootUnorderedAccessView(5, m_counter.Gpu());
        cmd->Dispatch(1, 1, 1);
        m_counter.UavBarrier(cmd);

        m_lightGrid.TransitionTo(cmd, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        m_indexList.TransitionTo(cmd, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        cmd->SetPipelineState(m_cullPSO.Get());
        cmd->SetComputeRootConstantBufferView(0, cullCbVA);
        cmd->SetComputeRootShaderResourceView(1, lightsVA);
        cmd->SetComputeRootShaderResourceView(2, m_aabb.Gpu());
        cmd->SetComputeRootUnorderedAccessView(3, m_lightGrid.Gpu());
        cmd->SetComputeRootUnorderedAccessView(4, m_indexList.Gpu());
        cmd->SetComputeRootUnorderedAccessView(5, m_counter.Gpu());
        cmd->Dispatch(Groups(m_froxels), 1, 1);
        m_lightGrid.UavBarrier(cmd); m_indexList.UavBarrier(cmd);
        m_lightGrid.TransitionTo(cmd, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        m_indexList.TransitionTo(cmd, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        prof.EndRegion(cmd, id);
    }

    // ---- 3. Forward shade (back buffer + depth already bound by BeginFrame) ----
    {
        const uint32_t id = prof.BeginRegion(cmd, "Shade");
        FrameCB fr = {};
        XMStoreFloat4x4(&fr.view, view);
        fr.grid[0] = grid[0]; fr.grid[1] = grid[1]; fr.grid[2] = grid[2];
        fr.mode = m_heatmap ? 1u : 0u;
        fr.znear = kNear; fr.zfar = kFar; fr.screen[0] = float(w); fr.screen[1] = float(h);
        fr.camPos[0] = ctx.cameraPos[0]; fr.camPos[1] = ctx.cameraPos[1]; fr.camPos[2] = ctx.cameraPos[2];
        fr.tileX = float(w) / float(m_tilesX); fr.tileY = float(h) / float(m_tilesY);
        const D3D12_GPU_VIRTUAL_ADDRESS frameVA = push(&fr, sizeof(fr));

        cmd->SetGraphicsRootSignature(m_shadeRS.Get());
        cmd->SetPipelineState(m_shadePSO.Get());
        cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        cmd->SetGraphicsRootConstantBufferView(1, frameVA);
        cmd->SetGraphicsRootShaderResourceView(2, lightsVA);
        cmd->SetGraphicsRootShaderResourceView(3, m_lightGrid.Gpu());
        cmd->SetGraphicsRootShaderResourceView(4, m_indexList.Gpu());

        auto drawBox = [&](XMMATRIX model, XMFLOAT4 albedo) {
            ObjCB o; XMStoreFloat4x4(&o.mvp, model * viewProj);
            XMStoreFloat4x4(&o.model, model); o.albedo = albedo;
            cmd->SetGraphicsRootConstantBufferView(0, push(&o, sizeof(o)));
            m_cube->Draw(cmd);
        };
        // Ground.
        drawBox(XMMatrixScaling(44.0f, 0.3f, 44.0f) * XMMatrixTranslation(0, -1.3f, 0),
                { 0.6f, 0.6f, 0.62f, 1.0f });
        // A field of boxes to catch the light pools.
        const int n = 7; const float sp = 5.0f, off = (n - 1) * 0.5f * sp;
        for (int gz = 0; gz < n; ++gz)
            for (int gx = 0; gx < n; ++gx) {
                const float hgt = 1.0f + 0.6f * float((gx * 3 + gz) % 4);
                drawBox(XMMatrixScaling(1.2f, hgt, 1.2f) *
                        XMMatrixTranslation(gx * sp - off, hgt * 0.5f - 1.15f, gz * sp - off),
                        { 0.7f, 0.7f, 0.72f, 1.0f });
            }
        prof.EndRegion(cmd, id);
    }
}

void ClusteredScene::OnImGui() {
    ImGui::SliderInt("Lights", &m_lightCount, 1, 2048);
    ImGui::Checkbox("Animate", &m_animate);
    ImGui::Checkbox("Light-count heatmap", &m_heatmap);
    ImGui::Separator();
    ImGui::Text("Froxel grid: %u x %u x %u = %u", m_tilesX, m_tilesY, kSlices, m_froxels);
    ImGui::TextDisabled("%d lights culled into per-froxel lists each frame.", m_lightCount);
    ImGui::TextDisabled("Heatmap: blue=few lights ... red=many (/32).");
}

bool ClusteredScene::PreferredCamera(float pos[3], float& yaw, float& pitch) const {
    pos[0] = 0.0f; pos[1] = 15.0f; pos[2] = -20.0f;
    yaw = 0.0f; pitch = -0.55f;
    return true;
}
