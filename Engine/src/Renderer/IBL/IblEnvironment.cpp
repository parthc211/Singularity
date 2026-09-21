#include "Renderer/IBL/IblEnvironment.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/RootSignatureBuilder.h"
#include "Renderer/DX12/ComputePipeline.h"
#include "Renderer/ShaderLibrary.h"
#include "Core/Logger.h"

#include <algorithm>

namespace SGE {

namespace {

// A DEFAULT-heap texture usable as both a UAV (compute writes) and an SRV
// (sampled later). Cubes are just 6-slice 2D arrays; the cube-ness is a view
// concept. No optimized clear value is allowed on a UAV texture.
ComPtr<ID3D12Resource> CreateGpuTexture(ID3D12Device* device, DXGI_FORMAT fmt,
                                        uint32_t size, uint32_t mips, uint32_t arraySize)
{
    D3D12_HEAP_PROPERTIES heap = {};
    heap.Type = D3D12_HEAP_TYPE_DEFAULT;

    D3D12_RESOURCE_DESC d = {};
    d.Dimension        = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    d.Width            = size;
    d.Height           = size;
    d.DepthOrArraySize = UINT16(arraySize);
    d.MipLevels        = UINT16(mips);
    d.Format           = fmt;
    d.SampleDesc.Count = 1;
    d.Flags            = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;

    ComPtr<ID3D12Resource> res;
    SGE_THROW_IF_FAILED(device->CreateCommittedResource(
        &heap, D3D12_HEAP_FLAG_NONE, &d,
        D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&res)));
    return res;
}

D3D12_RESOURCE_BARRIER Transition(ID3D12Resource* r, D3D12_RESOURCE_STATES before,
                                  D3D12_RESOURCE_STATES after)
{
    D3D12_RESOURCE_BARRIER b = {};
    b.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    b.Transition.pResource   = r;
    b.Transition.StateBefore = before;
    b.Transition.StateAfter  = after;
    b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    return b;
}

// Cube face UAV over one mip (all 6 slices), for a RWTexture2DArray write.
void MakeCubeMipUav(ID3D12Device* device, ID3D12Resource* res, DXGI_FORMAT fmt,
                    uint32_t mip, D3D12_CPU_DESCRIPTOR_HANDLE dst)
{
    D3D12_UNORDERED_ACCESS_VIEW_DESC u = {};
    u.Format                         = fmt;
    u.ViewDimension                  = D3D12_UAV_DIMENSION_TEXTURE2DARRAY;
    u.Texture2DArray.MipSlice        = mip;
    u.Texture2DArray.FirstArraySlice = 0;
    u.Texture2DArray.ArraySize       = 6;
    device->CreateUnorderedAccessView(res, nullptr, &u, dst);
}

void MakeTexCubeSrv(ID3D12Device* device, ID3D12Resource* res, DXGI_FORMAT fmt,
                    uint32_t mips, D3D12_CPU_DESCRIPTOR_HANDLE dst)
{
    D3D12_SHADER_RESOURCE_VIEW_DESC s = {};
    s.Format                        = fmt;
    s.ViewDimension                 = D3D12_SRV_DIMENSION_TEXTURECUBE;
    s.Shader4ComponentMapping       = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    s.TextureCube.MipLevels         = mips;
    s.TextureCube.MostDetailedMip   = 0;
    device->CreateShaderResourceView(res, &s, dst);
}

// A static linear-clamp sampler visible to ALL stages — the preset builders make
// it pixel-only, which a compute shader can't see.
D3D12_STATIC_SAMPLER_DESC LinearClampAll(UINT reg)
{
    D3D12_STATIC_SAMPLER_DESC s = {};
    s.Filter           = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    s.AddressU = s.AddressV = s.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    s.MaxLOD           = D3D12_FLOAT32_MAX;
    s.ShaderRegister   = reg;
    s.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    return s;
}

constexpr uint32_t Groups(uint32_t n) { return (n + 7) / 8; } // ceil(n/8) for numthreads(8,8,1)

} // namespace

bool IblEnvironment::Bake(ID3D12Device* device, ID3D12CommandQueue* queue,
                          ShaderLibrary& shaders, const IblBakeParams& params)
{
    try {
        Reset();

        // --- Resources ---
        m_env         = CreateGpuTexture(device, kHdrFormat, kEnvSize,        kEnvMips,       6);
        m_irradiance  = CreateGpuTexture(device, kHdrFormat, kIrradianceSize, 1,              6);
        m_prefiltered = CreateGpuTexture(device, kHdrFormat, kPrefilterSize,  kPrefilterMips, 6);
        m_brdfLut     = CreateGpuTexture(device, kLutFormat, kBrdfLutSize,    1,              1);

        // --- Root signatures ---
        RootSignature skyRS, irrRS, preRS, lutRS;
        const auto flags = D3D12_ROOT_SIGNATURE_FLAG_NONE;
        if (!RootSignatureBuilder().Constants(0, 6).UavTable(0, 1).Build(device, skyRS, flags))
            return false;
        if (!RootSignatureBuilder().Constants(0, 2).SrvTable(0, 1, D3D12_SHADER_VISIBILITY_ALL)
                 .UavTable(0, 1).Sampler(LinearClampAll(0)).Build(device, irrRS, flags))
            return false;
        if (!RootSignatureBuilder().Constants(0, 4).SrvTable(0, 1, D3D12_SHADER_VISIBILITY_ALL)
                 .UavTable(0, 1).Sampler(LinearClampAll(0)).Build(device, preRS, flags))
            return false;
        if (!RootSignatureBuilder().Constants(0, 1).UavTable(0, 1).Build(device, lutRS, flags))
            return false;

        // --- Compute pipelines ---
        ComputePipeline skyPSO, irrPSO, prePSO, lutPSO;
        auto skyCS = shaders.GetOrCompile(L"IblSky.hlsl",        "CSMain", "cs_6_0");
        auto irrCS = shaders.GetOrCompile(L"IblIrradiance.hlsl", "CSMain", "cs_6_0");
        auto preCS = shaders.GetOrCompile(L"IblPrefilter.hlsl",  "CSMain", "cs_6_0");
        auto lutCS = shaders.GetOrCompile(L"IblBrdfLut.hlsl",    "CSMain", "cs_6_0");
        if (!skyCS || !irrCS || !preCS || !lutCS) return false;
        if (!skyPSO.Create(device, skyRS.Get(), skyCS)) return false;
        if (!irrPSO.Create(device, irrRS.Get(), irrCS)) return false;
        if (!prePSO.Create(device, preRS.Get(), preCS)) return false;
        if (!lutPSO.Create(device, lutRS.Get(), lutCS)) return false;

        // --- Descriptor heap for the bake (SRVs/UAVs the tables point at) ---
        const uint32_t kSlots = 1 + kEnvMips + 1 + kPrefilterMips + 1; // envSrv + envUavs + irr + preUavs + lut
        D3D12_DESCRIPTOR_HEAP_DESC hd = {};
        hd.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
        hd.NumDescriptors = kSlots;
        hd.Flags          = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        ComPtr<ID3D12DescriptorHeap> heap;
        SGE_THROW_IF_FAILED(device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&heap)));
        const UINT inc = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
        auto cpu = [&](uint32_t i) { auto h = heap->GetCPUDescriptorHandleForHeapStart(); h.ptr += SIZE_T(i) * inc; return h; };
        auto gpu = [&](uint32_t i) { auto h = heap->GetGPUDescriptorHandleForHeapStart(); h.ptr += UINT64(i) * inc; return h; };

        // Slot assignment.
        uint32_t slot = 0;
        const uint32_t sEnvSrv = slot++;                         // TextureCube of env
        const uint32_t sEnvUav = slot; slot += kEnvMips;         // one UAV per env mip
        const uint32_t sIrrUav = slot++;
        const uint32_t sPreUav = slot; slot += kPrefilterMips;   // one UAV per prefiltered mip
        const uint32_t sLutUav = slot++;

        MakeTexCubeSrv(device, m_env.Get(), kHdrFormat, kEnvMips, cpu(sEnvSrv));
        for (uint32_t m = 0; m < kEnvMips; ++m)
            MakeCubeMipUav(device, m_env.Get(), kHdrFormat, m, cpu(sEnvUav + m));
        MakeCubeMipUav(device, m_irradiance.Get(), kHdrFormat, 0, cpu(sIrrUav));
        for (uint32_t m = 0; m < kPrefilterMips; ++m)
            MakeCubeMipUav(device, m_prefiltered.Get(), kHdrFormat, m, cpu(sPreUav + m));
        {
            D3D12_UNORDERED_ACCESS_VIEW_DESC u = {};
            u.Format        = kLutFormat;
            u.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
            device->CreateUnorderedAccessView(m_brdfLut.Get(), nullptr, &u, cpu(sLutUav));
        }

        // --- One-shot command list (own allocator/list, execute, fence) ---
        ComPtr<ID3D12CommandAllocator> allocator;
        ComPtr<ID3D12GraphicsCommandList> cmd;
        SGE_THROW_IF_FAILED(device->CreateCommandAllocator(
            D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&allocator)));
        SGE_THROW_IF_FAILED(device->CreateCommandList(
            0, D3D12_COMMAND_LIST_TYPE_DIRECT, allocator.Get(), nullptr, IID_PPV_ARGS(&cmd)));

        ID3D12DescriptorHeap* heaps[] = { heap.Get() };
        cmd->SetDescriptorHeaps(1, heaps);

        const D3D12_RESOURCE_STATES READ = D3D12_RESOURCE_STATES(
            D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE |
            D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

        // 1. Environment sky — bake every mip directly from the analytic sky.
        {
            auto b = Transition(m_env.Get(), D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            cmd->ResourceBarrier(1, &b);
            cmd->SetPipelineState(skyPSO.Get());
            cmd->SetComputeRootSignature(skyRS.Get());
            struct { uint32_t faceSize; float sunDir[3]; float sunIntensity; float exposure; } sky{};
            sky.sunDir[0] = params.sunDir[0]; sky.sunDir[1] = params.sunDir[1]; sky.sunDir[2] = params.sunDir[2];
            sky.sunIntensity = params.sunIntensity;
            sky.exposure     = params.exposure;
            for (uint32_t m = 0; m < kEnvMips; ++m) {
                const uint32_t sz = std::max(kEnvSize >> m, 1u);
                sky.faceSize = sz;
                cmd->SetComputeRoot32BitConstants(0, 6, &sky, 0);
                cmd->SetComputeRootDescriptorTable(1, gpu(sEnvUav + m));
                cmd->Dispatch(Groups(sz), Groups(sz), 6);
            }
            b = Transition(m_env.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, READ);
            cmd->ResourceBarrier(1, &b);
        }

        // 2. Irradiance — cosine convolution of the env cube.
        {
            auto b = Transition(m_irradiance.Get(), D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            cmd->ResourceBarrier(1, &b);
            cmd->SetPipelineState(irrPSO.Get());
            cmd->SetComputeRootSignature(irrRS.Get());
            uint32_t c[2] = { kIrradianceSize, 0 };
            cmd->SetComputeRoot32BitConstants(0, 2, c, 0);
            cmd->SetComputeRootDescriptorTable(1, gpu(sEnvSrv));
            cmd->SetComputeRootDescriptorTable(2, gpu(sIrrUav));
            cmd->Dispatch(Groups(kIrradianceSize), Groups(kIrradianceSize), 6);
            b = Transition(m_irradiance.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, READ);
            cmd->ResourceBarrier(1, &b);
        }

        // 3. Prefiltered env — one GGX-convolved mip per roughness level.
        {
            auto b = Transition(m_prefiltered.Get(), D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            cmd->ResourceBarrier(1, &b);
            cmd->SetPipelineState(prePSO.Get());
            cmd->SetComputeRootSignature(preRS.Get());
            cmd->SetComputeRootDescriptorTable(1, gpu(sEnvSrv));
            for (uint32_t m = 0; m < kPrefilterMips; ++m) {
                const uint32_t sz = std::max(kPrefilterSize >> m, 1u);
                struct { uint32_t faceSize; float roughness; uint32_t envSize; uint32_t samples; } pre{};
                pre.faceSize  = sz;
                pre.roughness = float(m) / float(kPrefilterMips - 1);
                pre.envSize   = kEnvSize;
                pre.samples   = (m == 0) ? 1u : 128u;
                cmd->SetComputeRoot32BitConstants(0, 4, &pre, 0);
                cmd->SetComputeRootDescriptorTable(2, gpu(sPreUav + m));
                cmd->Dispatch(Groups(sz), Groups(sz), 6);
            }
            b = Transition(m_prefiltered.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, READ);
            cmd->ResourceBarrier(1, &b);
        }

        // 4. BRDF integration LUT.
        {
            auto b = Transition(m_brdfLut.Get(), D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            cmd->ResourceBarrier(1, &b);
            cmd->SetPipelineState(lutPSO.Get());
            cmd->SetComputeRootSignature(lutRS.Get());
            uint32_t sz = kBrdfLutSize;
            cmd->SetComputeRoot32BitConstants(0, 1, &sz, 0);
            cmd->SetComputeRootDescriptorTable(1, gpu(sLutUav));
            cmd->Dispatch(Groups(kBrdfLutSize), Groups(kBrdfLutSize), 1);
            b = Transition(m_brdfLut.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, READ);
            cmd->ResourceBarrier(1, &b);
        }

        SGE_THROW_IF_FAILED(cmd->Close());
        ID3D12CommandList* lists[] = { cmd.Get() };
        queue->ExecuteCommandLists(1, lists);

        ComPtr<ID3D12Fence> fence;
        SGE_THROW_IF_FAILED(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence)));
        HANDLE event = CreateEvent(nullptr, FALSE, FALSE, nullptr);
        if (!event) throw std::runtime_error("IblEnvironment::Bake: CreateEvent failed");
        SGE_THROW_IF_FAILED(queue->Signal(fence.Get(), 1));
        SGE_THROW_IF_FAILED(fence->SetEventOnCompletion(1, event));
        WaitForSingleObject(event, INFINITE);
        CloseHandle(event);

        LogInfo("IBL environment baked.");
        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        Reset();
        return false;
    }
}

void IblEnvironment::CreateEnvSrvInto(ID3D12Device* device, D3D12_CPU_DESCRIPTOR_HANDLE dst) const
{
    MakeTexCubeSrv(device, m_env.Get(), kHdrFormat, kEnvMips, dst);
}
void IblEnvironment::CreateIrradianceSrvInto(ID3D12Device* device, D3D12_CPU_DESCRIPTOR_HANDLE dst) const
{
    MakeTexCubeSrv(device, m_irradiance.Get(), kHdrFormat, 1, dst);
}
void IblEnvironment::CreatePrefilteredSrvInto(ID3D12Device* device, D3D12_CPU_DESCRIPTOR_HANDLE dst) const
{
    MakeTexCubeSrv(device, m_prefiltered.Get(), kHdrFormat, kPrefilterMips, dst);
}
void IblEnvironment::CreateBrdfLutSrvInto(ID3D12Device* device, D3D12_CPU_DESCRIPTOR_HANDLE dst) const
{
    D3D12_SHADER_RESOURCE_VIEW_DESC s = {};
    s.Format                    = kLutFormat;
    s.ViewDimension             = D3D12_SRV_DIMENSION_TEXTURE2D;
    s.Shader4ComponentMapping   = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    s.Texture2D.MipLevels       = 1;
    device->CreateShaderResourceView(m_brdfLut.Get(), &s, dst);
}

void IblEnvironment::Reset()
{
    m_env.Reset();
    m_irradiance.Reset();
    m_prefiltered.Reset();
    m_brdfLut.Reset();
}

} // namespace SGE
