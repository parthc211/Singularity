#include "Scenes/RtScene.h"

#include "Renderer/Renderer.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"
#include "Renderer/DX12/RootSignatureBuilder.h"
#include "Core/Camera.h"
#include "Core/Logger.h"

#include "imgui.h"
#include <DirectXMath.h>
#include <cmath>
#include <cstring>
#include <vector>

using namespace SGE;
using namespace DirectX;

namespace {
// Per-TLAS-instance shading data, indexed in the hit shader by InstanceID().
// vbIndex/ibIndex are bindless heap slots: the hit shader fetches the hit mesh's
// vertices via ResourceDescriptorHeap[vbIndex] without the SBT knowing which mesh
// it hit. 32 bytes; must match struct InstanceData in RtScene.hlsl.
struct InstanceData {
    uint32_t vbIndex;
    uint32_t ibIndex;
    float    reflectivity;
    uint32_t _pad0;
    float    color[3];
    uint32_t _pad1;
};
struct SceneCB  { XMFLOAT4X4 invViewProj; XMFLOAT4 camPos; XMFLOAT4 sunDir; uint32_t bounces; float pad[3]; };

constexpr uint32_t kIdSize   = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES;        // 32
constexpr uint64_t kRayGenOff = 0;
constexpr uint64_t kMissOff   = 64;   // next 64-aligned region
constexpr uint64_t kHitOff    = 128;  // after 2 miss records of 32

// m_rtHeap descriptor slots (shader-visible CBV/SRV/UAV heap, directly indexed).
constexpr uint32_t kSlotOutUav = 0;  // RT output UAV   (ray-gen writes)
constexpr uint32_t kSlotOutSrv = 1;  // RT output SRV   (blit reads)
constexpr uint32_t kSlotCubeVB = 2;  // StructuredBuffer<MeshVertex>
constexpr uint32_t kSlotCubeIB = 3;  // Buffer<uint>
constexpr uint32_t kSlotSphVB  = 4;
constexpr uint32_t kSlotSphIB  = 5;
constexpr uint32_t kRtHeapSize = 6;

// The scene as a list of instances of the two SHARED meshes (0 = cube, 1 = sphere).
// Transforms map each unit mesh into place; the cube is [-0.5,0.5] so its scale is
// the full box size, and the unit sphere's scale is its radius. Used by both the
// per-instance material setup and the TLAS build, so it lives in one place.
struct SceneInst { int mesh; XMFLOAT4X4 xform; float col[3]; float refl; };

std::vector<SceneInst> SceneInstances() {
    auto M = [](XMMATRIX m){ XMFLOAT4X4 f; XMStoreFloat4x4(&f, m); return f; };
    return {
        { 0, M(XMMatrixScaling(30, 0.2f, 30)    * XMMatrixTranslation(0, -1.0f, 0)),          {0.5f,0.52f,0.55f}, 0.5f  }, // reflective floor
        { 0, M(XMMatrixScaling(1.6f,1.8f,1.6f) * XMMatrixTranslation(-3.0f,-0.1f, 1.0f)),    {0.9f,0.25f,0.25f}, 0.0f  }, // red box
        { 0, M(XMMatrixScaling(1.4f,1.4f,1.4f) * XMMatrixTranslation( 3.2f,-0.3f,-1.0f)),    {0.3f,0.85f,0.35f}, 0.0f  }, // green box
        { 1, M(XMMatrixScaling(1.2f,1.2f,1.2f) * XMMatrixTranslation( 0.0f, 0.2f, 0.0f)),    {0.9f,0.9f,0.92f},  0.85f }, // mirror sphere
        { 1, M(XMMatrixScaling(0.8f,0.8f,0.8f) * XMMatrixTranslation(-1.5f,-0.3f,-2.5f)),    {1.0f,0.82f,0.35f}, 0.6f  }, // gold sphere
        { 1, M(XMMatrixScaling(1.0f,1.0f,1.0f) * XMMatrixTranslation( 1.6f, 0.0f, 2.2f)),    {0.3f,0.45f,0.95f}, 0.0f  }, // blue sphere
    };
}

ComPtr<ID3D12Resource> CreateBuffer(ID3D12Device* dev, uint64_t size,
                                    D3D12_HEAP_TYPE heap, D3D12_RESOURCE_FLAGS flags,
                                    D3D12_RESOURCE_STATES state) {
    D3D12_HEAP_PROPERTIES hp = {}; hp.Type = heap;
    D3D12_RESOURCE_DESC d = {};
    d.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    d.Width = size; d.Height = 1; d.DepthOrArraySize = 1; d.MipLevels = 1;
    d.SampleDesc.Count = 1; d.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR; d.Flags = flags;
    ComPtr<ID3D12Resource> r;
    SGE_THROW_IF_FAILED(dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d, state, nullptr, IID_PPV_ARGS(&r)));
    return r;
}

void UavBarrier(ID3D12GraphicsCommandList* c, ID3D12Resource* r) {
    D3D12_RESOURCE_BARRIER b = {}; b.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV; b.UAV.pResource = r;
    c->ResourceBarrier(1, &b);
}
} // namespace

const char* RtScene::Description() const {
    return "Hardware ray tracing (DXR). The SHARED cube + sphere meshes are built "
           "into BLAS/TLAS acceleration structures (a TLAS instances them with "
           "per-instance transforms + materials); a ray-tracing pipeline + shader "
           "binding table drive DispatchRays. Each pixel traces a primary ray, a "
           "hard shadow ray to the sun, and recursive reflection rays. Reflections "
           "trace the REAL scene, so they show off-screen geometry that "
           "screen-space reflections can't. Move the sun and bounce count below.";
}

void RtScene::OnLoad(const DemoContext& ctx) {
    m_supported = ctx.renderer->SupportsRaytracing();
    if (!m_supported) return;

    m_shaders.Initialize(L"Shaders");
    if (!BuildGeometry(ctx)) return;
    if (!BuildAccelerationStructures(ctx)) return;
    if (!BuildPipeline(ctx)) return;
    if (!BuildShaderTable(ctx)) return;
    if (!CreateOutput(ctx, ctx.renderer->GetWidth(), ctx.renderer->GetHeight())) return;

    // Blit pipeline (RT output -> back buffer).
    if (!RootSignatureBuilder().SrvTable(0, 1).SamplerLinearClamp(0).Build(ctx.device, m_blitRS)) return;
    auto vs = m_shaders.GetOrCompile(L"RtBlit.hlsl", "VSMain", "vs_6_0");
    auto ps = m_shaders.GetOrCompile(L"RtBlit.hlsl", "PSMain", "ps_6_0");
    if (!vs || !ps) return;
    GraphicsPipelineDesc d; d.rootSignature = m_blitRS.Get(); d.vs = vs; d.ps = ps;
    d.depthEnable = false; d.rtvFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
    if (!m_blitPSO.Create(ctx.device, d)) return;

    m_ready = true;
}

void RtScene::OnUnload() {
    m_ready = false;
    m_blitPSO.Reset(); m_blitRS.Reset();
    m_rtHeap.Reset(); m_output.Reset();
    m_sbt.Reset(); m_rtpso.Reset(); m_globalRS.Reset();
    m_tlas.Reset(); m_cubeBlas.Reset(); m_sphereBlas.Reset(); m_instanceData.Reset();
    if (m_supported) m_shaders.Shutdown();
}

bool RtScene::BuildGeometry(const DemoContext& ctx) {
    if (!m_cube || !m_cube->IsValid() || !m_sphere || !m_sphere->IsValid()) {
        LogError("RtScene: shared cube/sphere mesh unavailable");
        return false;
    }

    // One shader-visible heap holds the RT output (UAV + SRV) and the two meshes'
    // vertex/index SRVs. The meshes' VB/IB are directly indexed by the hit shader
    // (ResourceDescriptorHeap[]) so a hit on ANY instance can fetch its vertices.
    D3D12_DESCRIPTOR_HEAP_DESC hd = {};
    hd.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    hd.NumDescriptors = kRtHeapSize;
    hd.Flags          = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    if (FAILED(ctx.device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&m_rtHeap)))) return false;
    m_rtHeapInc = ctx.device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

    auto cpu = [&](uint32_t i){ auto h = m_rtHeap->GetCPUDescriptorHandleForHeapStart(); h.ptr += SIZE_T(i) * m_rtHeapInc; return h; };

    // Vertex buffer -> StructuredBuffer<MeshVertex> SRV (stride 48).
    auto vbSrv = [&](const Mesh* m, uint32_t slot){
        D3D12_SHADER_RESOURCE_VIEW_DESC s = {};
        s.Format                     = DXGI_FORMAT_UNKNOWN;
        s.ViewDimension              = D3D12_SRV_DIMENSION_BUFFER;
        s.Shader4ComponentMapping    = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        s.Buffer.NumElements         = m->VertexCount();
        s.Buffer.StructureByteStride = m->VertexStride();
        ctx.device->CreateShaderResourceView(m->VertexResource(), &s, cpu(slot));
    };
    // Index buffer -> typed Buffer<uint> SRV.
    auto ibSrv = [&](const Mesh* m, uint32_t slot){
        D3D12_SHADER_RESOURCE_VIEW_DESC s = {};
        s.Format                  = DXGI_FORMAT_R32_UINT;
        s.ViewDimension           = D3D12_SRV_DIMENSION_BUFFER;
        s.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        s.Buffer.NumElements      = m->IndexCount();
        ctx.device->CreateShaderResourceView(m->IndexResource(), &s, cpu(slot));
    };
    vbSrv(m_cube,   kSlotCubeVB); ibSrv(m_cube,   kSlotCubeIB);
    vbSrv(m_sphere, kSlotSphVB);  ibSrv(m_sphere, kSlotSphIB);

    // Per-instance shading data (bindless VB/IB indices + material), indexed by
    // InstanceID() in the hit shader.
    const auto scene = SceneInstances();
    m_instanceCount = uint32_t(scene.size());
    m_triangleCount = 0;
    std::vector<InstanceData> data(scene.size());
    for (size_t i = 0; i < scene.size(); ++i) {
        const bool  isCube = scene[i].mesh == 0;
        const Mesh* m      = isCube ? m_cube : m_sphere;
        data[i].vbIndex      = isCube ? kSlotCubeVB : kSlotSphVB;
        data[i].ibIndex      = isCube ? kSlotCubeIB : kSlotSphIB;
        data[i].reflectivity = scene[i].refl;
        data[i].color[0] = scene[i].col[0];
        data[i].color[1] = scene[i].col[1];
        data[i].color[2] = scene[i].col[2];
        m_triangleCount += m->IndexCount() / 3;
    }
    m_instanceData = CreateBuffer(ctx.device, data.size() * sizeof(InstanceData),
                                  D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_FLAG_NONE,
                                  D3D12_RESOURCE_STATE_GENERIC_READ);
    void* p = nullptr;
    m_instanceData->Map(0, nullptr, &p);
    std::memcpy(p, data.data(), data.size() * sizeof(InstanceData));
    m_instanceData->Unmap(0, nullptr);
    return true;
}

bool RtScene::BuildAccelerationStructures(const DemoContext& ctx) {
    ID3D12Device* dev = ctx.device;
    ComPtr<ID3D12Device5> dev5;
    if (FAILED(dev->QueryInterface(IID_PPV_ARGS(&dev5)))) return false;

    const auto AS   = D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE;
    const auto UAVF = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;

    // --- One BLAS per shared mesh, built straight from the mesh's VRAM VB/IB
    //     (indexed triangles) — the exact point of exposing the GPU addresses. ---
    auto geoDesc = [](const Mesh* m){
        D3D12_RAYTRACING_GEOMETRY_DESC g = {};
        g.Type  = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
        g.Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;
        g.Triangles.VertexBuffer.StartAddress  = m->VertexAddress();
        g.Triangles.VertexBuffer.StrideInBytes = m->VertexStride();
        g.Triangles.VertexCount  = m->VertexCount();
        g.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT;  // position is the first 12 bytes
        g.Triangles.IndexBuffer  = m->IndexAddress();
        g.Triangles.IndexCount   = m->IndexCount();
        g.Triangles.IndexFormat  = DXGI_FORMAT_R32_UINT;
        return g;
    };
    const D3D12_RAYTRACING_GEOMETRY_DESC cubeGeo = geoDesc(m_cube);
    const D3D12_RAYTRACING_GEOMETRY_DESC sphGeo  = geoDesc(m_sphere);

    auto blasInputs = [](const D3D12_RAYTRACING_GEOMETRY_DESC* g){
        D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS in = {};
        in.Type           = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
        in.DescsLayout    = D3D12_ELEMENTS_LAYOUT_ARRAY;
        in.Flags          = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;
        in.NumDescs       = 1;
        in.pGeometryDescs = g;
        return in;
    };
    const D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS cubeIn = blasInputs(&cubeGeo);
    const D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS sphIn  = blasInputs(&sphGeo);
    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO cubeInfo = {}, sphInfo = {};
    dev5->GetRaytracingAccelerationStructurePrebuildInfo(&cubeIn, &cubeInfo);
    dev5->GetRaytracingAccelerationStructurePrebuildInfo(&sphIn,  &sphInfo);

    // --- TLAS over m_instanceCount instances of those two BLASes. ---
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS tlasIn = {};
    tlasIn.Type        = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
    tlasIn.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    tlasIn.Flags       = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;
    tlasIn.NumDescs    = m_instanceCount;
    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO tlasInfo = {};
    dev5->GetRaytracingAccelerationStructurePrebuildInfo(&tlasIn, &tlasInfo);

    m_cubeBlas   = CreateBuffer(dev, cubeInfo.ResultDataMaxSizeInBytes, D3D12_HEAP_TYPE_DEFAULT, UAVF, AS);
    m_sphereBlas = CreateBuffer(dev, sphInfo.ResultDataMaxSizeInBytes,  D3D12_HEAP_TYPE_DEFAULT, UAVF, AS);
    m_tlas       = CreateBuffer(dev, tlasInfo.ResultDataMaxSizeInBytes, D3D12_HEAP_TYPE_DEFAULT, UAVF, AS);
    auto cubeScratch = CreateBuffer(dev, cubeInfo.ScratchDataSizeInBytes, D3D12_HEAP_TYPE_DEFAULT, UAVF, D3D12_RESOURCE_STATE_COMMON);
    auto sphScratch  = CreateBuffer(dev, sphInfo.ScratchDataSizeInBytes,  D3D12_HEAP_TYPE_DEFAULT, UAVF, D3D12_RESOURCE_STATE_COMMON);
    auto tlasScratch = CreateBuffer(dev, tlasInfo.ScratchDataSizeInBytes, D3D12_HEAP_TYPE_DEFAULT, UAVF, D3D12_RESOURCE_STATE_COMMON);

    // Instance descriptors: per-instance transform (baked into the TLAS), an
    // InstanceID the hit shader uses to look up its material/mesh, and the BLAS it
    // references. DXR wants the 3x4 transform row-major of the column-vector
    // mapping, i.e. the first three rows of the transpose of the row-vector matrix.
    const auto scene = SceneInstances();
    std::vector<D3D12_RAYTRACING_INSTANCE_DESC> instDescs(scene.size());
    for (size_t i = 0; i < scene.size(); ++i) {
        const XMMATRIX Mrow = XMLoadFloat4x4(&scene[i].xform);
        XMFLOAT4X4 mt; XMStoreFloat4x4(&mt, XMMatrixTranspose(Mrow));
        D3D12_RAYTRACING_INSTANCE_DESC d = {};
        std::memcpy(d.Transform, mt.m, sizeof(d.Transform));  // first 3 rows (12 floats)
        d.InstanceID                          = uint32_t(i);
        d.InstanceMask                        = 0xFF;
        d.InstanceContributionToHitGroupIndex = 0;
        d.AccelerationStructure = (scene[i].mesh == 0 ? m_cubeBlas : m_sphereBlas)->GetGPUVirtualAddress();
        instDescs[i] = d;
    }
    auto instBuf = CreateBuffer(dev, instDescs.size() * sizeof(D3D12_RAYTRACING_INSTANCE_DESC),
                                D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_FLAG_NONE, D3D12_RESOURCE_STATE_GENERIC_READ);
    {
        void* p = nullptr;
        instBuf->Map(0, nullptr, &p);
        std::memcpy(p, instDescs.data(), instDescs.size() * sizeof(D3D12_RAYTRACING_INSTANCE_DESC));
        instBuf->Unmap(0, nullptr);
    }
    tlasIn.InstanceDescs = instBuf->GetGPUVirtualAddress();

    // One-shot command list: build both BLASes, then the TLAS (which reads the BLAS
    // results — the UAV barriers order them).
    ComPtr<ID3D12CommandAllocator> alloc;
    ComPtr<ID3D12GraphicsCommandList> cmd;
    SGE_THROW_IF_FAILED(dev->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&alloc)));
    SGE_THROW_IF_FAILED(dev->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, alloc.Get(), nullptr, IID_PPV_ARGS(&cmd)));
    ComPtr<ID3D12GraphicsCommandList4> cmd4;
    if (FAILED(cmd->QueryInterface(IID_PPV_ARGS(&cmd4)))) return false;

    auto buildAs = [&](const D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS& in,
                       ID3D12Resource* dst, ID3D12Resource* scratch){
        D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC bd = {};
        bd.Inputs                           = in;
        bd.ScratchAccelerationStructureData = scratch->GetGPUVirtualAddress();
        bd.DestAccelerationStructureData    = dst->GetGPUVirtualAddress();
        cmd4->BuildRaytracingAccelerationStructure(&bd, 0, nullptr);
        UavBarrier(cmd4.Get(), dst);
    };
    buildAs(cubeIn, m_cubeBlas.Get(),   cubeScratch.Get());
    buildAs(sphIn,  m_sphereBlas.Get(), sphScratch.Get());
    buildAs(tlasIn, m_tlas.Get(),       tlasScratch.Get());

    SGE_THROW_IF_FAILED(cmd->Close());
    ID3D12CommandList* lists[] = { cmd.Get() };
    ctx.renderer->GetCommandQueue()->ExecuteCommandLists(1, lists);

    // Fence-wait so the scratch/instance buffers stay alive through the build.
    ComPtr<ID3D12Fence> fence;
    SGE_THROW_IF_FAILED(dev->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence)));
    HANDLE ev = CreateEvent(nullptr, FALSE, FALSE, nullptr);
    ctx.renderer->GetCommandQueue()->Signal(fence.Get(), 1);
    fence->SetEventOnCompletion(1, ev); WaitForSingleObject(ev, INFINITE); CloseHandle(ev);
    return true;
}

bool RtScene::BuildPipeline(const DemoContext& ctx) {
    ID3D12Device* dev = ctx.device;
    // Global root signature: UAV table (output) + TLAS SRV + instances SRV + CBV.
    // DirectlyIndexed() lets the hit shader reach the meshes' VB/IB SRVs through
    // ResourceDescriptorHeap[] (the bindless vertex fetch).
    if (!RootSignatureBuilder().UavTable(0, 1).Srv(0).Srv(1).Cbv(0).DirectlyIndexed()
             .Build(dev, m_globalRS, D3D12_ROOT_SIGNATURE_FLAG_NONE))
        return false;

    ComPtr<ID3D12Device5> dev5;
    if (FAILED(dev->QueryInterface(IID_PPV_ARGS(&dev5)))) return false;

    // lib_6_6: the hit shader uses ResourceDescriptorHeap[] (SM 6.6 dynamic
    // resources) for the bindless vertex fetch.
    auto lib = m_shaders.GetOrCompile(L"RtScene.hlsl", "", "lib_6_6");
    if (!lib) return false;
    const auto bc = lib->GetBytecode();

    D3D12_EXPORT_DESC exports[] = {
        { L"RayGen", nullptr, D3D12_EXPORT_FLAG_NONE },
        { L"Miss", nullptr, D3D12_EXPORT_FLAG_NONE },
        { L"ShadowMiss", nullptr, D3D12_EXPORT_FLAG_NONE },
        { L"ClosestHit", nullptr, D3D12_EXPORT_FLAG_NONE },
    };
    D3D12_DXIL_LIBRARY_DESC libDesc = {};
    libDesc.DXILLibrary = bc; libDesc.NumExports = _countof(exports); libDesc.pExports = exports;

    D3D12_HIT_GROUP_DESC hit = {};
    hit.HitGroupExport = L"HitGroup";
    hit.Type = D3D12_HIT_GROUP_TYPE_TRIANGLES;
    hit.ClosestHitShaderImport = L"ClosestHit";

    D3D12_RAYTRACING_SHADER_CONFIG shaderCfg = {};
    shaderCfg.MaxPayloadSizeInBytes   = 16; // float3 colour + uint depth
    shaderCfg.MaxAttributeSizeInBytes = 8;  // barycentrics

    // Worst-case recursion: primary (1) + a reflection chain of gBounces, each
    // hit also tracing a shadow ray on top, so the deepest TraceRay sits at
    // (maxBounces + 2). The slider caps bounces at 4 -> need 6; 8 leaves headroom.
    // Exceeding this at runtime is undefined behaviour (GPU fault / device removed).
    D3D12_RAYTRACING_PIPELINE_CONFIG pipeCfg = {};
    pipeCfg.MaxTraceRecursionDepth = 8;

    ID3D12RootSignature* grs = m_globalRS.Get();
    D3D12_GLOBAL_ROOT_SIGNATURE globalRS = { grs };

    D3D12_STATE_SUBOBJECT subs[5] = {
        { D3D12_STATE_SUBOBJECT_TYPE_DXIL_LIBRARY, &libDesc },
        { D3D12_STATE_SUBOBJECT_TYPE_HIT_GROUP, &hit },
        { D3D12_STATE_SUBOBJECT_TYPE_RAYTRACING_SHADER_CONFIG, &shaderCfg },
        { D3D12_STATE_SUBOBJECT_TYPE_RAYTRACING_PIPELINE_CONFIG, &pipeCfg },
        { D3D12_STATE_SUBOBJECT_TYPE_GLOBAL_ROOT_SIGNATURE, &globalRS },
    };
    D3D12_STATE_OBJECT_DESC so = { D3D12_STATE_OBJECT_TYPE_RAYTRACING_PIPELINE, _countof(subs), subs };
    if (FAILED(dev5->CreateStateObject(&so, IID_PPV_ARGS(&m_rtpso)))) return false;
    return true;
}

bool RtScene::BuildShaderTable(const DemoContext& ctx) {
    ComPtr<ID3D12StateObjectProperties> props;
    if (FAILED(m_rtpso.As(&props))) return false;

    m_sbtMissOffset = kMissOff; m_sbtHitOffset = kHitOff;
    m_sbt = CreateBuffer(ctx.device, 256, D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_FLAG_NONE,
                         D3D12_RESOURCE_STATE_GENERIC_READ);
    uint8_t* p = nullptr;
    m_sbt->Map(0, nullptr, reinterpret_cast<void**>(&p));
    std::memset(p, 0, 256);
    std::memcpy(p + kRayGenOff,          props->GetShaderIdentifier(L"RayGen"),     kIdSize);
    std::memcpy(p + kMissOff,            props->GetShaderIdentifier(L"Miss"),       kIdSize);
    std::memcpy(p + kMissOff + kIdSize,  props->GetShaderIdentifier(L"ShadowMiss"), kIdSize);
    std::memcpy(p + kHitOff,             props->GetShaderIdentifier(L"HitGroup"),   kIdSize);
    m_sbt->Unmap(0, nullptr);
    return true;
}

bool RtScene::CreateOutput(const DemoContext& ctx, uint32_t w, uint32_t h) {
    m_w = w; m_h = h;
    D3D12_HEAP_PROPERTIES hp = {}; hp.Type = D3D12_HEAP_TYPE_DEFAULT;
    D3D12_RESOURCE_DESC d = {};
    d.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    d.Width = w; d.Height = h; d.DepthOrArraySize = 1; d.MipLevels = 1;
    d.Format = DXGI_FORMAT_R8G8B8A8_UNORM; d.SampleDesc.Count = 1;
    d.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
    m_output.Reset();
    if (FAILED(ctx.device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d,
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&m_output))))
        return false;

    // m_rtHeap is created in BuildGeometry (it also holds the mesh SRVs); this only
    // (re)writes the output UAV/SRV slots, so it survives a resize.
    auto cpu = [&](uint32_t i){ auto hh = m_rtHeap->GetCPUDescriptorHandleForHeapStart(); hh.ptr += SIZE_T(i)*m_rtHeapInc; return hh; };
    D3D12_UNORDERED_ACCESS_VIEW_DESC uav = {}; uav.Format = d.Format; uav.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    ctx.device->CreateUnorderedAccessView(m_output.Get(), nullptr, &uav, cpu(kSlotOutUav));
    D3D12_SHADER_RESOURCE_VIEW_DESC srv = {}; srv.Format = d.Format; srv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    srv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING; srv.Texture2D.MipLevels = 1;
    ctx.device->CreateShaderResourceView(m_output.Get(), &srv, cpu(kSlotOutSrv));
    return true;
}

void RtScene::OnUpdate(const DemoContext& ctx) { if (m_spinSun) m_time += ctx.dt; }

void RtScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;
    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;
    auto& prof = r->GetProfiler();

    const uint32_t w = r->GetWidth(), h = r->GetHeight();
    if (w != m_w || h != m_h) { r->WaitForGPU(); CreateOutput(ctx, w, h); }

    ComPtr<ID3D12GraphicsCommandList4> cmd4;
    if (FAILED(cmd->QueryInterface(IID_PPV_ARGS(&cmd4)))) return;

    // Scene constants.
    const XMMATRIX invVP = XMMatrixInverse(nullptr, ctx.camera->GetViewProjection());
    const float sy = m_spinSun ? m_time * 0.3f : m_sunYaw;
    SceneCB cb = {};
    XMStoreFloat4x4(&cb.invViewProj, invVP);
    cb.camPos = { ctx.cameraPos[0], ctx.cameraPos[1], ctx.cameraPos[2], 1 };
    cb.sunDir = { cosf(m_sunPitch) * sinf(sy), sinf(m_sunPitch), cosf(m_sunPitch) * cosf(sy), 0 };
    cb.bounces = uint32_t(m_bounces);
    auto cbAlloc = ctx.objectCB->Allocate(sizeof(cb));
    std::memcpy(cbAlloc.Cpu, &cb, sizeof(cb));

    // ---- DispatchRays ----
    {
        const uint32_t id = prof.BeginRegion(cmd, "RT Dispatch");
        ID3D12DescriptorHeap* heaps[] = { m_rtHeap.Get() };
        cmd->SetDescriptorHeaps(1, heaps);
        cmd->SetComputeRootSignature(m_globalRS.Get());
        cmd->SetComputeRootDescriptorTable(0, m_rtHeap->GetGPUDescriptorHandleForHeapStart());  // output UAV (slot 0)
        cmd->SetComputeRootShaderResourceView(1, m_tlas->GetGPUVirtualAddress());
        cmd->SetComputeRootShaderResourceView(2, m_instanceData->GetGPUVirtualAddress());
        cmd->SetComputeRootConstantBufferView(3, cbAlloc.Gpu);
        cmd4->SetPipelineState1(m_rtpso.Get());

        D3D12_DISPATCH_RAYS_DESC dr = {};
        const D3D12_GPU_VIRTUAL_ADDRESS sbt = m_sbt->GetGPUVirtualAddress();
        dr.RayGenerationShaderRecord = { sbt + kRayGenOff, kIdSize };
        dr.MissShaderTable           = { sbt + m_sbtMissOffset, 2 * kIdSize, kIdSize };
        dr.HitGroupTable             = { sbt + m_sbtHitOffset, kIdSize, kIdSize };
        dr.Width = w; dr.Height = h; dr.Depth = 1;
        cmd4->DispatchRays(&dr);
        prof.EndRegion(cmd, id);
    }

    // ---- Blit RT output -> back buffer ----
    {
        const uint32_t id = prof.BeginRegion(cmd, "RT Blit");
        D3D12_RESOURCE_BARRIER b = {};
        b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        b.Transition.pResource = m_output.Get();
        b.Transition.StateBefore = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        b.Transition.StateAfter  = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
        b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        cmd->ResourceBarrier(1, &b);

        r->BindBackBufferTargets(cmd);
        cmd->SetGraphicsRootSignature(m_blitRS.Get());
        cmd->SetPipelineState(m_blitPSO.Get());
        ID3D12DescriptorHeap* heaps[] = { m_rtHeap.Get() };
        cmd->SetDescriptorHeaps(1, heaps);
        auto gpu1 = m_rtHeap->GetGPUDescriptorHandleForHeapStart(); gpu1.ptr += UINT64(kSlotOutSrv) * m_rtHeapInc;
        cmd->SetGraphicsRootDescriptorTable(0, gpu1);
        cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        cmd->IASetVertexBuffers(0, 0, nullptr);
        cmd->DrawInstanced(3, 1, 0, 0);

        std::swap(b.Transition.StateBefore, b.Transition.StateAfter);
        cmd->ResourceBarrier(1, &b);   // back to UAV for next frame's dispatch
        prof.EndRegion(cmd, id);
    }
}

void RtScene::OnImGui() {
    if (!m_supported) {
        ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.3f, 1),
                           "DXR unsupported: this GPU/driver has no raytracing tier 1.0.");
        return;
    }
    ImGui::SliderInt("Reflection bounces", &m_bounces, 0, 4);
    ImGui::SliderFloat("Sun yaw",   &m_sunYaw,   -3.14f, 3.14f, "%.2f");
    ImGui::SliderFloat("Sun pitch", &m_sunPitch,  0.2f, 1.5f,  "%.2f");
    ImGui::Checkbox("Spin sun", &m_spinSun);
    ImGui::Separator();
    ImGui::TextDisabled("%u triangles, %u instances, 2 BLAS + TLAS, recursive DispatchRays.",
                        m_triangleCount, m_instanceCount);
    ImGui::TextDisabled("Shared cube + sphere meshes; hit shader fetches vertices bindlessly.");
    ImGui::TextDisabled("Reflections trace the real scene (unlike SSR).");
}

bool RtScene::PreferredCamera(float pos[3], float& yaw, float& pitch) const {
    pos[0] = 0.0f; pos[1] = 1.8f; pos[2] = -7.5f;
    yaw = 0.0f; pitch = -0.12f;
    return true;
}
