#include "Scenes/GpuDrivenScene.h"

#include "Renderer/Renderer.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"
#include "Renderer/DX12/RootSignatureBuilder.h"
#include "Renderer/MeshLoader.h"
#include "Core/Camera.h"
#include "Core/Logger.h"

#include "imgui.h"
#include <DirectXMath.h>
#include <cmath>
#include <cstring>
#include <random>
#include <vector>

using namespace SGE;
using namespace DirectX;

namespace {
struct Instance   { XMFLOAT4X4 model; XMFLOAT4 color; XMFLOAT4 sphere; };   // 96 bytes
struct IndirectCommand {                                                    // 24 bytes
    uint32_t instanceIndex;
    uint32_t indexCount, instanceCount, startIndex;
    int32_t  baseVertex;
    uint32_t startInstance;
};
struct FrameCB { XMFLOAT4X4 viewProj; XMFLOAT4 lightDir; };
struct CullCB  { XMFLOAT4 frustum[6]; uint32_t instanceCount, indexCount, cullEnabled, pad; };

ComPtr<ID3D12Resource> UploadBuffer(ID3D12Device* dev, const void* data, size_t size) {
    D3D12_HEAP_PROPERTIES hp = {}; hp.Type = D3D12_HEAP_TYPE_UPLOAD;
    D3D12_RESOURCE_DESC d = {};
    d.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER; d.Width = size; d.Height = 1;
    d.DepthOrArraySize = 1; d.MipLevels = 1; d.SampleDesc.Count = 1; d.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    ComPtr<ID3D12Resource> r;
    SGE_THROW_IF_FAILED(dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&r)));
    if (data) { void* p; r->Map(0, nullptr, &p); std::memcpy(p, data, size); r->Unmap(0, nullptr); }
    return r;
}

// Six normalized frustum planes from a row-vector view-projection (clip = v*M).
void ExtractFrustum(const XMFLOAT4X4& m, XMFLOAT4 out[6]) {
    auto pl = [](float a, float b, float c, float d) {
        const float l = sqrtf(a*a + b*b + c*c); return XMFLOAT4(a/l, b/l, c/l, d/l);
    };
    out[0] = pl(m._14+m._11, m._24+m._21, m._34+m._31, m._44+m._41); // left
    out[1] = pl(m._14-m._11, m._24-m._21, m._34-m._31, m._44-m._41); // right
    out[2] = pl(m._14+m._12, m._24+m._22, m._34+m._32, m._44+m._42); // bottom
    out[3] = pl(m._14-m._12, m._24-m._22, m._34-m._32, m._44-m._42); // top
    out[4] = pl(m._13,       m._23,       m._33,       m._43);       // near (z>=0)
    out[5] = pl(m._14-m._13, m._24-m._23, m._34-m._33, m._44-m._43); // far
}
} // namespace

const char* GpuDrivenScene::Description() const {
    return "GPU-driven rendering. Thousands of cube instances live in one buffer; "
           "a compute pass frustum-culls them and atomic-compacts the surviving "
           "DrawIndexedInstanced commands into an indirect-args buffer, which a "
           "single ExecuteIndirect draws. The CPU issues zero per-object draws — "
           "the GPU decides what's visible. Toggle culling off and watch the "
           "drawn count jump to the full total.";
}

void GpuDrivenScene::OnLoad(const DemoContext& ctx) {
    m_shaders.Initialize(L"Shaders");
    if (!BuildGeometry(ctx)) return;
    BuildInstances(ctx);
    if (!BuildPipelines(ctx)) return;

    // Indirect command list + survivor counter + read-back.
    m_commands.Create(ctx.device, uint64_t(m_instanceCount) * sizeof(IndirectCommand), true);
    m_counter.Create(ctx.device, 256, true);
    D3D12_HEAP_PROPERTIES rb = {}; rb.Type = D3D12_HEAP_TYPE_READBACK;
    D3D12_RESOURCE_DESC rd = {};
    rd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER; rd.Width = FrameCount * 4; rd.Height = 1;
    rd.DepthOrArraySize = 1; rd.MipLevels = 1; rd.SampleDesc.Count = 1; rd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    SGE_THROW_IF_FAILED(ctx.device->CreateCommittedResource(&rb, D3D12_HEAP_FLAG_NONE, &rd,
        D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&m_readback)));

    // Command signature: a root constant (instance index) + the draw args.
    D3D12_INDIRECT_ARGUMENT_DESC args[2] = {};
    args[0].Type = D3D12_INDIRECT_ARGUMENT_TYPE_CONSTANT;
    args[0].Constant.RootParameterIndex = 0;
    args[0].Constant.Num32BitValuesToSet = 1;
    args[1].Type = D3D12_INDIRECT_ARGUMENT_TYPE_DRAW_INDEXED;
    D3D12_COMMAND_SIGNATURE_DESC cs = {};
    cs.ByteStride = sizeof(IndirectCommand); cs.NumArgumentDescs = 2; cs.pArgumentDescs = args;
    if (FAILED(ctx.device->CreateCommandSignature(&cs, m_drawRS.Get(), IID_PPV_ARGS(&m_cmdSig)))) return;

    m_ready = true;
}

void GpuDrivenScene::OnUnload() {
    m_ready = false;
    m_cmdSig.Reset(); m_readback.Reset();
    m_commands.Reset(); m_counter.Reset();
    m_cube.Reset(); m_instances.Reset();
    m_drawPSO.Reset(); m_cullPSO.Reset(); m_resetPSO.Reset();
    m_drawRS.Reset(); m_cullRS.Reset();
    m_shaders.Shutdown();
}

bool GpuDrivenScene::BuildGeometry(const DemoContext& ctx) {
    // Reuse the engine's shared unit cube asset (cube.obj) instead of hand-authoring
    // geometry in-scene. Driving it from one ExecuteIndirect is exactly what the
    // Mesh geometry-resource API makes possible: Mesh exposes the VB/IB views and
    // index count the indirect path binds (see Mesh.h). Because the asset's winding
    // is already correct, the whole class of hand-rolled winding bugs can't recur.
    // The per-face debug colour is now derived from the (axis-aligned) object-space
    // normal in the shader, so no custom vertex attribute is needed.
    std::vector<MeshVertex> verts;
    std::vector<uint32_t>   indices;
    if (!LoadOBJ("Assets/cube.obj", verts, indices)) {
        LogError("GpuDrivenScene: failed to load Assets/cube.obj");
        return false;
    }
    return m_cube.Upload(ctx.device, ctx.renderer->GetCommandQueue(), verts, indices);
}

void GpuDrivenScene::BuildInstances(const DemoContext& ctx) {
    const int N = 120;                       // 120 x 120 = 14,400 instances
    const float spacing = 2.0f, off = (N-1)*0.5f*spacing;
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> hgt(0.4f, 3.0f), col(0.3f, 1.0f), jit(-0.4f, 0.4f);
    std::vector<Instance> inst; inst.reserve(size_t(N)*N);
    for (int z = 0; z < N; ++z) for (int x = 0; x < N; ++x) {
        const float h = hgt(rng);
        const float px = x*spacing - off + jit(rng), pz = z*spacing - off + jit(rng), py = h*0.5f - 1.0f;
        Instance in;
        XMStoreFloat4x4(&in.model, XMMatrixScaling(0.8f, h, 0.8f) * XMMatrixTranslation(px, py, pz));
        in.color  = { col(rng), col(rng), col(rng), 1.0f };
        const float radius = 0.5f * sqrtf(0.8f*0.8f + h*h + 0.8f*0.8f); // enclosing sphere
        in.sphere = { px, py, pz, radius };
        inst.push_back(in);
    }
    m_instanceCount = uint32_t(inst.size());
    m_instances = UploadBuffer(ctx.device, inst.data(), inst.size() * sizeof(Instance));
}

bool GpuDrivenScene::BuildPipelines(const DemoContext& ctx) {
    ID3D12Device* dev = ctx.device;
    const auto NONE = D3D12_ROOT_SIGNATURE_FLAG_NONE;

    // Cull: cbuffer + instances SRV + commands/counter UAV.
    if (!RootSignatureBuilder().Cbv(0).Srv(0).Uav(0).Uav(1).Build(dev, m_cullRS, NONE)) return false;
    auto cull  = m_shaders.GetOrCompile(L"GpuDrivenCull.hlsl", "CSMain",  "cs_6_0");
    auto reset = m_shaders.GetOrCompile(L"GpuDrivenCull.hlsl", "CSReset", "cs_6_0");
    if (!cull || !reset) return false;
    if (!m_cullPSO.Create(dev, m_cullRS.Get(), cull)) return false;
    if (!m_resetPSO.Create(dev, m_cullRS.Get(), reset)) return false;

    // Draw: per-draw root constant (instance index) + frame CBV + instances SRV.
    if (!RootSignatureBuilder().Constants(0, 1).Cbv(1).Srv(0).Build(dev, m_drawRS)) return false;
    auto vs = m_shaders.GetOrCompile(L"GpuDriven.hlsl", "VSMain", "vs_6_0");
    auto ps = m_shaders.GetOrCompile(L"GpuDriven.hlsl", "PSMain", "ps_6_0");
    if (!vs || !ps) return false;
    GraphicsPipelineDesc d; d.rootSignature = m_drawRS.Get(); d.vs = vs; d.ps = ps;
    d.depthEnable = true; d.dsvFormat = DXGI_FORMAT_D32_FLOAT;
    return m_drawPSO.Create(dev, d);
}

void GpuDrivenScene::OnUpdate(const DemoContext& ctx) { m_time += ctx.dt; }

void GpuDrivenScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;
    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;
    auto& prof = r->GetProfiler();
    const uint32_t frame = r->GetFrameIndex();

    // Read the survivor count this slot held FrameCount frames ago.
    {
        const D3D12_RANGE rr{ frame * 4, frame * 4 + 4 };
        void* p = nullptr;
        if (SUCCEEDED(m_readback->Map(0, &rr, &p))) {
            m_drawn = reinterpret_cast<const uint32_t*>(static_cast<const uint8_t*>(p))[frame];
            D3D12_RANGE nw{ 0, 0 }; m_readback->Unmap(0, &nw);
        }
    }

    const XMMATRIX viewProj = ctx.camera->GetViewProjection();
    XMFLOAT4X4 vp; XMStoreFloat4x4(&vp, viewProj);

    FrameCB fr; fr.viewProj = vp; fr.lightDir = { 0.4f, 0.8f, -0.4f, m_faceColors ? 1.0f : 0.0f };
    CullCB cull = {}; ExtractFrustum(vp, cull.frustum);
    cull.instanceCount = m_instanceCount; cull.indexCount = m_cube.IndexCount();
    cull.cullEnabled = m_cull ? 1u : 0u;
    auto frameVA = ctx.objectCB->Allocate(sizeof(fr));   std::memcpy(frameVA.Cpu, &fr, sizeof(fr));
    auto cullVA  = ctx.objectCB->Allocate(sizeof(cull)); std::memcpy(cullVA.Cpu, &cull, sizeof(cull));

    // ---- GPU cull -> indirect command list ----
    {
        const uint32_t id = prof.BeginRegion(cmd, "GPU Cull");
        m_counter.TransitionTo(cmd, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        cmd->SetComputeRootSignature(m_cullRS.Get());
        cmd->SetPipelineState(m_resetPSO.Get());
        cmd->SetComputeRootUnorderedAccessView(3, m_counter.Gpu());
        cmd->Dispatch(1, 1, 1);
        m_counter.UavBarrier(cmd);

        m_commands.TransitionTo(cmd, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        cmd->SetPipelineState(m_cullPSO.Get());
        cmd->SetComputeRootConstantBufferView(0, cullVA.Gpu);
        cmd->SetComputeRootShaderResourceView(1, m_instances->GetGPUVirtualAddress());
        cmd->SetComputeRootUnorderedAccessView(2, m_commands.Gpu());
        cmd->SetComputeRootUnorderedAccessView(3, m_counter.Gpu());
        cmd->Dispatch((m_instanceCount + 63) / 64, 1, 1);
        m_commands.UavBarrier(cmd); m_counter.UavBarrier(cmd);

        // Copy the count out for the HUD, then hand both buffers to ExecuteIndirect.
        m_counter.TransitionTo(cmd, D3D12_RESOURCE_STATE_COPY_SOURCE);
        cmd->CopyBufferRegion(m_readback.Get(), frame * 4, m_counter.Resource(), 0, 4);
        m_counter.TransitionTo(cmd, D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
        m_commands.TransitionTo(cmd, D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
        prof.EndRegion(cmd, id);
    }

    // ---- ExecuteIndirect: draw the survivors ----
    {
        const uint32_t id = prof.BeginRegion(cmd, "ExecuteIndirect");
        r->BindBackBufferTargets(cmd);
        cmd->SetGraphicsRootSignature(m_drawRS.Get());
        cmd->SetPipelineState(m_drawPSO.Get());
        cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        const auto vbv = m_cube.VertexView();
        const auto ibv = m_cube.IndexView();
        cmd->IASetVertexBuffers(0, 1, &vbv);
        cmd->IASetIndexBuffer(&ibv);
        cmd->SetGraphicsRootConstantBufferView(1, frameVA.Gpu);
        cmd->SetGraphicsRootShaderResourceView(2, m_instances->GetGPUVirtualAddress());
        cmd->ExecuteIndirect(m_cmdSig.Get(), m_instanceCount, m_commands.Resource(), 0,
                             m_counter.Resource(), 0);
        prof.EndRegion(cmd, id);
    }
}

void GpuDrivenScene::OnImGui() {
    ImGui::Checkbox("Frustum cull (GPU)", &m_cull);
    ImGui::Checkbox("Debug: colour each face", &m_faceColors);
    ImGui::TextDisabled("+X red, -X green, top blue, bottom YELLOW, +Z magenta, -Z cyan.");
    ImGui::Separator();
    ImGui::Text("Drawn: %u / %u", m_drawn, m_instanceCount);
    const float pct = m_instanceCount ? 100.0f * float(m_drawn) / float(m_instanceCount) : 0.0f;
    ImGui::ProgressBar(m_instanceCount ? float(m_drawn) / float(m_instanceCount) : 0.0f,
                       ImVec2(-1, 0), nullptr);
    ImGui::TextDisabled("%.0f%% survive culling. One ExecuteIndirect, zero per-object CPU draws.", pct);
}

bool GpuDrivenScene::PreferredCamera(float pos[3], float& yaw, float& pitch) const {
    pos[0] = 0.0f; pos[1] = 6.0f; pos[2] = -10.0f;
    yaw = 0.0f; pitch = -0.25f;
    return true;
}
