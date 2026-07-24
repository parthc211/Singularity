#include "Renderer/DX12/RenderTexture.h"
#include "Core/Logger.h"
#include <cstring>

namespace SGE {

bool RenderTexture::Create(ID3D12Device* device, uint32_t width, uint32_t height,
                           DXGI_FORMAT format, const float clearColor[4]) {
    try {
        Reset();
        m_width = width; m_height = height; m_format = format;
        std::memcpy(m_clear, clearColor, sizeof(m_clear));

        D3D12_HEAP_PROPERTIES heapProps = {};
        heapProps.Type = D3D12_HEAP_TYPE_DEFAULT;

        D3D12_RESOURCE_DESC desc = {};
        desc.Dimension        = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        desc.Width            = width;
        desc.Height           = height;
        desc.DepthOrArraySize = 1;
        desc.MipLevels        = 1;
        desc.Format           = format;
        desc.SampleDesc.Count = 1;
        desc.Flags            = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;

        D3D12_CLEAR_VALUE clear = {};
        clear.Format = format;
        std::memcpy(clear.Color, clearColor, sizeof(clear.Color));

        SGE_THROW_IF_FAILED(device->CreateCommittedResource(
            &heapProps, D3D12_HEAP_FLAG_NONE, &desc,
            D3D12_RESOURCE_STATE_COMMON, &clear, IID_PPV_ARGS(&m_tex)));

        D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc = {};
        rtvHeapDesc.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
        rtvHeapDesc.NumDescriptors = 1;
        SGE_THROW_IF_FAILED(device->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(&m_rtvHeap)));
        device->CreateRenderTargetView(m_tex.Get(), nullptr,
                                       m_rtvHeap->GetCPUDescriptorHandleForHeapStart());

        m_state = D3D12_RESOURCE_STATE_COMMON;
        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

void RenderTexture::Reset() {
    m_tex.Reset();
    m_rtvHeap.Reset();
    m_state = D3D12_RESOURCE_STATE_COMMON;
    m_width = m_height = 0;
}

void RenderTexture::CreateSrvInto(ID3D12Device* device, D3D12_CPU_DESCRIPTOR_HANDLE dst) const {
    D3D12_SHADER_RESOURCE_VIEW_DESC srv = {};
    srv.Format                  = m_format;
    srv.ViewDimension           = D3D12_SRV_DIMENSION_TEXTURE2D;
    srv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srv.Texture2D.MipLevels     = 1;
    device->CreateShaderResourceView(m_tex.Get(), &srv, dst);
}

void RenderTexture::TransitionTo(ID3D12GraphicsCommandList* cmd, D3D12_RESOURCE_STATES state) {
    if (m_state == state) return;
    D3D12_RESOURCE_BARRIER b = {};
    b.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    b.Transition.pResource   = m_tex.Get();
    b.Transition.StateBefore = m_state;
    b.Transition.StateAfter  = state;
    b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    cmd->ResourceBarrier(1, &b);
    m_state = state;
}

void RenderTexture::ClearRtv(ID3D12GraphicsCommandList* cmd) {
    cmd->ClearRenderTargetView(Rtv(), m_clear, 0, nullptr);
}

} // namespace SGE
