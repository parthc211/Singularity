#include "Scenes/MeshShaderScene.h"

#include "Renderer/Renderer.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"
#include "Renderer/DX12/RootSignatureBuilder.h"
#include "Renderer/Meshlet.h"
#include "Core/Camera.h"

#include "imgui.h"
#include <DirectXMath.h>
#include <cmath>
#include <cstring>
#include <vector>

using namespace SGE;
using namespace DirectX;

namespace {
struct FrameCB {
    XMFLOAT4X4 viewProj;
    XMFLOAT4X4 model;
    XMFLOAT4   frustum[6];
    XMFLOAT4   camPos;
    uint32_t   meshletCount, cullEnabled, colorMode, pad;
};

// Six normalized world-space frustum planes from a row-vector view-projection.
void ExtractFrustum(const XMFLOAT4X4& m, XMFLOAT4 out[6]) {
    auto pl = [](float a, float b, float c, float d) {
        const float l = sqrtf(a*a + b*b + c*c); return XMFLOAT4(a/l, b/l, c/l, d/l);
    };
    out[0] = pl(m._14+m._11, m._24+m._21, m._34+m._31, m._44+m._41); // left
    out[1] = pl(m._14-m._11, m._24-m._21, m._34-m._31, m._44-m._41); // right
    out[2] = pl(m._14+m._12, m._24+m._22, m._34+m._32, m._44+m._42); // bottom
    out[3] = pl(m._14-m._12, m._24-m._22, m._34-m._32, m._44-m._42); // top
    out[4] = pl(m._13,       m._23,       m._33,       m._43);       // near
    out[5] = pl(m._14-m._13, m._24-m._23, m._34-m._33, m._44-m._43); // far
}

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

// Dense UV sphere, CW-from-outside winding (matches the engine's front-face
// convention), smooth normals — enough triangles to span many meshlets.
void BuildSphere(int lat, int lon, float radius,
                 std::vector<MeshVertex>& v, std::vector<uint32_t>& idx) {
    for (int i = 0; i <= lat; ++i) {
        const float phi = XM_PI * float(i) / float(lat);
        const float y = cosf(phi), sy = sinf(phi);
        for (int j = 0; j <= lon; ++j) {
            const float th = XM_2PI * float(j) / float(lon);
            const float x = sy * cosf(th), z = sy * sinf(th);
            MeshVertex mv = {};
            mv.position[0] = x*radius; mv.position[1] = y*radius; mv.position[2] = z*radius;
            mv.normal[0]   = x;        mv.normal[1]   = y;        mv.normal[2]   = z;
            mv.texCoord[0] = float(j)/float(lon); mv.texCoord[1] = float(i)/float(lat);
            mv.tangent[0]  = -sinf(th); mv.tangent[1] = 0.0f; mv.tangent[2] = cosf(th); mv.tangent[3] = 1.0f;
            v.push_back(mv);
        }
    }
    const int stride = lon + 1;
    for (int i = 0; i < lat; ++i) for (int j = 0; j < lon; ++j) {
        const uint32_t a = uint32_t(i*stride + j), b = a+1, c = a+uint32_t(stride), d = c+1;
        idx.insert(idx.end(), { a, b, d }); // CW from outside
        idx.insert(idx.end(), { a, d, c });
    }
}
} // namespace

const char* MeshShaderScene::Description() const {
    return "Mesh shaders (amplification -> mesh -> pixel). A dense sphere is "
           "partitioned into meshlets (clusters of <=64 verts / <=124 tris); one "
           "DispatchMesh launches an amplification shader that frustum- and "
           "normal-cone-culls whole meshlets, then dispatches only the survivors "
           "as mesh-shader groups that emit triangles directly — no input "
           "assembler, no index buffer. Tint each meshlet to see the partition.";
}

void MeshShaderScene::OnLoad(const DemoContext& ctx) {
    m_supported = ctx.renderer->SupportsMeshShaders();
    if (!m_supported) return;

    m_shaders.Initialize(L"Shaders");
    if (!BuildGeometry(ctx)) return;
    if (!BuildPipeline(ctx)) return;
    m_ready = true;
}

void MeshShaderScene::OnUnload() {
    m_ready = false;
    m_pso.Reset(); m_rootSig.Reset();
    m_meshlets.Reset(); m_bounds.Reset(); m_vertexIndices.Reset(); m_primIndices.Reset();
    m_mesh.Reset();
    if (m_supported) m_shaders.Shutdown();
}

bool MeshShaderScene::BuildGeometry(const DemoContext& ctx) {
    std::vector<MeshVertex> verts;
    std::vector<uint32_t>   indices;
    BuildSphere(48, 96, 1.6f, verts, indices);
    if (!m_mesh.Upload(ctx.device, ctx.renderer->GetCommandQueue(), verts, indices))
        return false;

    const MeshletMesh ml = BuildMeshlets(verts, indices);
    if (!ml.IsValid()) return false;
    m_meshletCount  = ml.MeshletCount();
    m_triangleCount = uint32_t(indices.size() / 3);

    m_meshlets      = UploadBuffer(ctx.device, ml.meshlets.data(),      ml.meshlets.size()      * sizeof(Meshlet));
    m_bounds        = UploadBuffer(ctx.device, ml.bounds.data(),        ml.bounds.size()        * sizeof(MeshletBounds));
    m_vertexIndices = UploadBuffer(ctx.device, ml.vertexIndices.data(), ml.vertexIndices.size() * sizeof(uint32_t));
    m_primIndices   = UploadBuffer(ctx.device, ml.primIndices.data(),   ml.primIndices.size()   * sizeof(uint32_t));
    return true;
}

bool MeshShaderScene::BuildPipeline(const DemoContext& ctx) {
    ID3D12Device* dev = ctx.device;
    // Frame CBV + 5 root SRVs (vertices, meshlets, bounds, vertex indices, prim
    // indices). No input-assembler flag — the mesh shader fetches geometry itself.
    if (!RootSignatureBuilder().Cbv(0).Srv(0).Srv(1).Srv(2).Srv(3).Srv(4)
             .Build(dev, m_rootSig, D3D12_ROOT_SIGNATURE_FLAG_NONE))
        return false;

    auto as = m_shaders.GetOrCompile(L"MeshShader.hlsl", "ASMain", "as_6_5");
    auto ms = m_shaders.GetOrCompile(L"MeshShader.hlsl", "MSMain", "ms_6_5");
    auto ps = m_shaders.GetOrCompile(L"MeshShader.hlsl", "PSMain", "ps_6_5");
    if (!as || !ms || !ps) return false;

    MeshPipelineDesc d;
    d.rootSignature = m_rootSig.Get();
    d.as = as; d.ms = ms; d.ps = ps;
    d.rtvFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
    d.dsvFormat = DXGI_FORMAT_D32_FLOAT;
    d.depthEnable = true;
    return m_pso.Create(dev, d);
}

void MeshShaderScene::OnUpdate(const DemoContext& ctx) { if (m_spin) m_time += ctx.dt; }

void MeshShaderScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;
    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;
    auto& prof = r->GetProfiler();

    ComPtr<ID3D12GraphicsCommandList6> cmd6;
    if (FAILED(cmd->QueryInterface(IID_PPV_ARGS(&cmd6)))) return;

    const XMMATRIX view = ctx.camera->GetViewProjection();
    const XMMATRIX model = XMMatrixRotationX(0.35f) * XMMatrixRotationY(m_time * 0.4f);

    FrameCB fr = {};
    XMStoreFloat4x4(&fr.viewProj, view);
    XMStoreFloat4x4(&fr.model, model);
    XMFLOAT4X4 vp; XMStoreFloat4x4(&vp, view);
    ExtractFrustum(vp, fr.frustum);
    fr.camPos = { ctx.cameraPos[0], ctx.cameraPos[1], ctx.cameraPos[2], 1.0f };
    fr.meshletCount = m_meshletCount;
    fr.cullEnabled  = m_cull ? 1u : 0u;
    fr.colorMode    = m_colorMeshlets ? 1u : 0u;
    auto frameVA = ctx.objectCB->Allocate(sizeof(fr));
    std::memcpy(frameVA.Cpu, &fr, sizeof(fr));

    const uint32_t id = prof.BeginRegion(cmd, "Mesh Shading");
    r->BindBackBufferTargets(cmd);
    cmd->SetGraphicsRootSignature(m_rootSig.Get());
    cmd->SetPipelineState(m_pso.Get());
    cmd->SetGraphicsRootConstantBufferView(0, frameVA.Gpu);
    cmd->SetGraphicsRootShaderResourceView(1, m_mesh.VertexAddress());
    cmd->SetGraphicsRootShaderResourceView(2, m_meshlets->GetGPUVirtualAddress());
    cmd->SetGraphicsRootShaderResourceView(3, m_bounds->GetGPUVirtualAddress());
    cmd->SetGraphicsRootShaderResourceView(4, m_vertexIndices->GetGPUVirtualAddress());
    cmd->SetGraphicsRootShaderResourceView(5, m_primIndices->GetGPUVirtualAddress());

    // One amplification group per 32 meshlets; the AS culls and dispatches survivors.
    cmd6->DispatchMesh((m_meshletCount + 31) / 32, 1, 1);
    prof.EndRegion(cmd, id);
}

void MeshShaderScene::OnImGui() {
    if (!m_supported) {
        ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.3f, 1),
                           "Mesh shaders unsupported: this GPU/driver has no mesh-shader tier 1.");
        return;
    }
    ImGui::Checkbox("Per-meshlet cull (AS: frustum + normal cone)", &m_cull);
    ImGui::Checkbox("Tint each meshlet", &m_colorMeshlets);
    ImGui::Checkbox("Spin", &m_spin);
    ImGui::Separator();
    ImGui::Text("Meshlets: %u   Triangles: %u", m_meshletCount, m_triangleCount);
    ImGui::TextDisabled("<=64 verts / <=124 tris per meshlet. One DispatchMesh, no IA, no index buffer.");
    ImGui::TextDisabled("The amplification shader drops whole back-facing / off-screen clusters.");
}

bool MeshShaderScene::PreferredCamera(float pos[3], float& yaw, float& pitch) const {
    pos[0] = 0.0f; pos[1] = 0.0f; pos[2] = -4.5f;
    yaw = 0.0f; pitch = 0.0f;
    return true;
}
