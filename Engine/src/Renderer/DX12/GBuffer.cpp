#include "Renderer/DX12/GBuffer.h"
#include "Core/Logger.h"

namespace SGE {

// Albedo packs into 8-bit UNORM; normal and world position need signed/large
// range, so they use 16-bit float targets.
const DXGI_FORMAT GBuffer::kFormats[GBuffer::kCount] = {
    DXGI_FORMAT_R8G8B8A8_UNORM,     // Albedo (base colour)
    DXGI_FORMAT_R16G16B16A16_FLOAT, // world-space Normal
    DXGI_FORMAT_R16G16B16A16_FLOAT, // world-space Position
};

DXGI_FORMAT GBuffer::Format(uint32_t i) { return kFormats[i]; }

bool GBuffer::Create(ID3D12Device* device, uint32_t width, uint32_t height) {
    try {
        Reset();
        m_width  = width;
        m_height = height;

        D3D12_HEAP_PROPERTIES heapProps = {};
        heapProps.Type = D3D12_HEAP_TYPE_DEFAULT; // GPU-local

        // RTV heap (CPU-only: render targets are written by the output-merger, the
        // GPU doesn't sample through these handles).
        D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc = {};
        rtvHeapDesc.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
        rtvHeapDesc.NumDescriptors = kCount;
        rtvHeapDesc.Flags          = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
        SGE_THROW_IF_FAILED(device->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(&m_rtvHeap)));
        m_rtvStride = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

        // SRV heap (shader-visible: the lighting pixel shader samples through it).
        D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
        srvHeapDesc.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
        srvHeapDesc.NumDescriptors = kCount;
        srvHeapDesc.Flags          = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        SGE_THROW_IF_FAILED(device->CreateDescriptorHeap(&srvHeapDesc, IID_PPV_ARGS(&m_srvHeap)));
        const uint32_t srvStride = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

        D3D12_CPU_DESCRIPTOR_HANDLE rtvHandle = m_rtvHeap->GetCPUDescriptorHandleForHeapStart();
        D3D12_CPU_DESCRIPTOR_HANDLE srvHandle = m_srvHeap->GetCPUDescriptorHandleForHeapStart();

        for (uint32_t i = 0; i < kCount; ++i) {
            D3D12_RESOURCE_DESC texDesc = {};
            texDesc.Dimension        = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
            texDesc.Width            = width;
            texDesc.Height           = height;
            texDesc.DepthOrArraySize = 1;
            texDesc.MipLevels        = 1;
            texDesc.Format           = kFormats[i];
            texDesc.SampleDesc.Count = 1;
            texDesc.Layout           = D3D12_TEXTURE_LAYOUT_UNKNOWN;
            texDesc.Flags            = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;

            // A matching optimized clear value avoids a perf warning and is what
            // ClearRenderTargetView must use (0,0,0,0 here).
            D3D12_CLEAR_VALUE clear = {};
            clear.Format = kFormats[i];

            SGE_THROW_IF_FAILED(device->CreateCommittedResource(
                &heapProps, D3D12_HEAP_FLAG_NONE, &texDesc,
                D3D12_RESOURCE_STATE_COMMON, &clear, IID_PPV_ARGS(&m_textures[i])));

            device->CreateRenderTargetView(m_textures[i].Get(), nullptr, rtvHandle);

            D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
            srvDesc.Format                  = kFormats[i];
            srvDesc.ViewDimension           = D3D12_SRV_DIMENSION_TEXTURE2D;
            srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
            srvDesc.Texture2D.MipLevels     = 1;
            device->CreateShaderResourceView(m_textures[i].Get(), &srvDesc, srvHandle);

            rtvHandle.ptr += m_rtvStride;
            srvHandle.ptr += srvStride;
        }

        m_state = D3D12_RESOURCE_STATE_COMMON;
        LogInfo("GBuffer created.");
        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

void GBuffer::Reset() {
    for (auto& t : m_textures) t.Reset();
    m_rtvHeap.Reset();
    m_srvHeap.Reset();
    m_rtvStride = 0;
    m_state     = D3D12_RESOURCE_STATE_COMMON;
    m_width = m_height = 0;
}

D3D12_CPU_DESCRIPTOR_HANDLE GBuffer::Rtv(uint32_t i) const {
    D3D12_CPU_DESCRIPTOR_HANDLE h = m_rtvHeap->GetCPUDescriptorHandleForHeapStart();
    h.ptr += static_cast<SIZE_T>(i) * m_rtvStride;
    return h;
}

D3D12_GPU_DESCRIPTOR_HANDLE GBuffer::SrvTable() const {
    return m_srvHeap->GetGPUDescriptorHandleForHeapStart();
}

void GBuffer::TransitionAll(ID3D12GraphicsCommandList* cmd, D3D12_RESOURCE_STATES to) {
    if (m_state == to) return;
    D3D12_RESOURCE_BARRIER barriers[kCount] = {};
    for (uint32_t i = 0; i < kCount; ++i) {
        barriers[i].Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barriers[i].Transition.pResource   = m_textures[i].Get();
        barriers[i].Transition.StateBefore = m_state;
        barriers[i].Transition.StateAfter  = to;
        barriers[i].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    }
    cmd->ResourceBarrier(kCount, barriers);
    m_state = to;
}

void GBuffer::TransitionToRenderTargets(ID3D12GraphicsCommandList* cmd) {
    TransitionAll(cmd, D3D12_RESOURCE_STATE_RENDER_TARGET);
}

void GBuffer::TransitionToShaderResources(ID3D12GraphicsCommandList* cmd) {
    TransitionAll(cmd, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
}

void GBuffer::ClearRenderTargets(ID3D12GraphicsCommandList* cmd) {
    const float black[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    for (uint32_t i = 0; i < kCount; ++i)
        cmd->ClearRenderTargetView(Rtv(i), black, 0, nullptr);
}

} // namespace SGE
