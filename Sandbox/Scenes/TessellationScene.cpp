#include "Scenes/TessellationScene.h"

#include "Core/Camera.h"
#include "Renderer/Renderer.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"

#include "imgui.h"
#include <DirectXMath.h>
#include <vector>
#include <cstring>

using namespace SGE;
using namespace DirectX;

namespace {
constexpr int   kPatchesPerSide = 32;
constexpr float kTerrainSize    = 64.0f;

// Must match TessParams in Tessellation.hlsl.
struct TessCB {
    XMFLOAT4X4 ViewProj;
    XMFLOAT3   CameraPos; float MaxTess;
    float      LodScale;  float HeightScale; float NoiseFreq; float _pad;
};
}

const char* TessellationScene::Description() const {
    return "Hardware tessellation: a 32x32 grid of quad patches is subdivided on "
           "the GPU by the hull/domain shaders, finer near the camera (LOD), and "
           "each vertex is displaced by procedural fBm noise into terrain. Toggle "
           "wireframe to watch the triangle density follow the camera.";
}

void TessellationScene::BuildGrid(const DemoContext& ctx) {
    const int   N       = kPatchesPerSide;
    const float spacing = kTerrainSize / N;
    const float origin  = -kTerrainSize * 0.5f;

    std::vector<XMFLOAT3> verts;
    verts.reserve((N + 1) * (N + 1));
    for (int j = 0; j <= N; ++j)
        for (int i = 0; i <= N; ++i)
            verts.push_back({ origin + i * spacing, 0.0f, origin + j * spacing });

    auto vid = [N](int i, int j) { return static_cast<std::uint32_t>(j * (N + 1) + i); };
    std::vector<std::uint32_t> indices;
    indices.reserve(static_cast<std::size_t>(N) * N * 4);
    for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i) {
            indices.push_back(vid(i,     j));     // control point 0 (u0,v0)
            indices.push_back(vid(i + 1, j));     // 1 (u1,v0)
            indices.push_back(vid(i + 1, j + 1)); // 2 (u1,v1)
            indices.push_back(vid(i,     j + 1)); // 3 (u0,v1)
        }

    GpuHeap& heap = ctx.renderer->GetGeometryHeap();
    m_vb.Upload(ctx.device, heap, verts.data(),
                static_cast<std::uint32_t>(verts.size() * sizeof(XMFLOAT3)), sizeof(XMFLOAT3));
    m_ib.Upload(ctx.device, heap, indices.data(), static_cast<std::uint32_t>(indices.size()));
    m_indexCount = static_cast<std::uint32_t>(indices.size());
}

bool TessellationScene::BuildPipelines(const DemoContext& ctx) {
    ID3D12Device* device = ctx.device;
    m_shaders.Initialize(L"Shaders");

    // One root CBV at b0, visible to all stages (VS/HS/DS read transforms + LOD).
    D3D12_ROOT_PARAMETER cbv      = {};
    cbv.ParameterType             = D3D12_ROOT_PARAMETER_TYPE_CBV;
    cbv.Descriptor.ShaderRegister = 0;
    cbv.ShaderVisibility          = D3D12_SHADER_VISIBILITY_ALL;
    if (!m_rootSig.Create(device, &cbv, 1))
        return false;

    auto vs = m_shaders.GetOrCompile(L"Tessellation.hlsl", "VSMain", "vs_6_0");
    auto hs = m_shaders.GetOrCompile(L"Tessellation.hlsl", "HSMain", "hs_6_0");
    auto ds = m_shaders.GetOrCompile(L"Tessellation.hlsl", "DSMain", "ds_6_0");
    auto ps = m_shaders.GetOrCompile(L"Tessellation.hlsl", "PSMain", "ps_6_0");
    if (!vs || !hs || !ds || !ps)
        return false;

    GraphicsPipelineDesc d;
    d.rootSignature = m_rootSig.Get();
    d.vs = vs; d.hs = hs; d.ds = ds; d.ps = ps;
    d.topologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_PATCH; // required with HS/DS
    d.cullMode     = D3D12_CULL_MODE_NONE;                // terrain seen from either side
    d.depthEnable  = true;
    d.rtvFormat    = DXGI_FORMAT_R8G8B8A8_UNORM;
    if (!m_solidPSO.Create(device, d)) return false;

    d.wireframe = true;
    return m_wirePSO.Create(device, d);
}

void TessellationScene::OnLoad(const DemoContext& ctx) {
    BuildGrid(ctx);
    m_ready = BuildPipelines(ctx);
}

void TessellationScene::OnUnload() {
    m_vb.Reset();       // returns its placed resource to the geometry heap
    m_ib.Reset();
    m_solidPSO.Reset();
    m_wirePSO.Reset();
    m_rootSig.Reset();
    m_shaders.Shutdown();
    m_ready = false;
}

void TessellationScene::OnRender(const DemoContext& ctx) {
    if (!m_ready)
        return;
    ID3D12GraphicsCommandList* cmd = ctx.cmd;

    TessCB cb;
    XMStoreFloat4x4(&cb.ViewProj, ctx.camera->GetViewProjection()); // no transpose (engine convention)
    cb.CameraPos   = { ctx.cameraPos[0], ctx.cameraPos[1], ctx.cameraPos[2] };
    cb.MaxTess     = m_maxTess;
    cb.LodScale    = m_lodScale;
    cb.HeightScale = m_heightScale;
    cb.NoiseFreq   = m_noiseFreq;

    const auto alloc = ctx.objectCB->Allocate(sizeof(TessCB));
    if (!alloc.Cpu)
        return;
    std::memcpy(alloc.Cpu, &cb, sizeof(cb));

    // Back buffer + depth + viewport are already bound by Renderer::BeginFrame.
    cmd->SetGraphicsRootSignature(m_rootSig.Get());
    cmd->SetPipelineState(m_wireframe ? m_wirePSO.Get() : m_solidPSO.Get());
    cmd->SetGraphicsRootConstantBufferView(0, alloc.Gpu);
    cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_4_CONTROL_POINT_PATCHLIST);

    D3D12_VERTEX_BUFFER_VIEW vbv = m_vb.GetView();
    D3D12_INDEX_BUFFER_VIEW  ibv = m_ib.GetView();
    cmd->IASetVertexBuffers(0, 1, &vbv);
    cmd->IASetIndexBuffer(&ibv);
    cmd->DrawIndexedInstanced(m_indexCount, 1, 0, 0, 0);
}

void TessellationScene::OnImGui() {
    ImGui::SliderFloat("Max tess factor", &m_maxTess, 1.0f, 64.0f, "%.0f");
    ImGui::SliderFloat("LOD scale",       &m_lodScale, 20.0f, 600.0f, "%.0f");
    ImGui::SliderFloat("Height scale",    &m_heightScale, 0.0f, 25.0f, "%.1f");
    ImGui::SliderFloat("Noise frequency", &m_noiseFreq, 0.005f, 0.2f, "%.3f");
    ImGui::Checkbox("Wireframe", &m_wireframe);
    ImGui::TextDisabled("Hold RMB + WASD to fly; subdivision follows the camera.");
}
