#include "Renderer/DX12/CascadedShadowMap.h"
#include "Core/Logger.h"

namespace SGE {

bool CascadedShadowMap::Create(ID3D12Device* device, uint32_t size, uint32_t cascadeCount) {
    try {
        Reset();
        m_size  = size;
        m_count = (cascadeCount > kMaxCascades) ? kMaxCascades : cascadeCount;

        D3D12_HEAP_PROPERTIES heapProps = {};
        heapProps.Type = D3D12_HEAP_TYPE_DEFAULT;

        D3D12_RESOURCE_DESC desc = {};
        desc.Dimension        = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        desc.Width            = size;
        desc.Height           = size;
        desc.DepthOrArraySize = static_cast<UINT16>(m_count); // array slices
        desc.MipLevels        = 1;
        desc.Format           = DXGI_FORMAT_R32_TYPELESS;
        desc.SampleDesc.Count = 1;
        desc.Flags            = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;

        D3D12_CLEAR_VALUE clear = {};
        clear.Format             = DXGI_FORMAT_D32_FLOAT;
        clear.DepthStencil.Depth = 1.0f;

        SGE_THROW_IF_FAILED(device->CreateCommittedResource(
            &heapProps, D3D12_HEAP_FLAG_NONE, &desc,
            D3D12_RESOURCE_STATE_COMMON, &clear, IID_PPV_ARGS(&m_texture)));

        // One DSV per slice.
        D3D12_DESCRIPTOR_HEAP_DESC dsvHeapDesc = {};
        dsvHeapDesc.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
        dsvHeapDesc.NumDescriptors = m_count;
        SGE_THROW_IF_FAILED(device->CreateDescriptorHeap(&dsvHeapDesc, IID_PPV_ARGS(&m_dsvHeap)));
        m_dsvStride = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_DSV);

        D3D12_CPU_DESCRIPTOR_HANDLE dsv = m_dsvHeap->GetCPUDescriptorHandleForHeapStart();
        for (uint32_t i = 0; i < m_count; ++i) {
            D3D12_DEPTH_STENCIL_VIEW_DESC d = {};
            d.Format                         = DXGI_FORMAT_D32_FLOAT;
            d.ViewDimension                  = D3D12_DSV_DIMENSION_TEXTURE2DARRAY;
            d.Texture2DArray.MipSlice        = 0;
            d.Texture2DArray.FirstArraySlice = i;
            d.Texture2DArray.ArraySize       = 1;
            device->CreateDepthStencilView(m_texture.Get(), &d, dsv);
            dsv.ptr += m_dsvStride;
        }

        // One array SRV covering all slices.
        D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
        srvHeapDesc.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
        srvHeapDesc.NumDescriptors = 1;
        srvHeapDesc.Flags          = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        SGE_THROW_IF_FAILED(device->CreateDescriptorHeap(&srvHeapDesc, IID_PPV_ARGS(&m_srvHeap)));

        D3D12_SHADER_RESOURCE_VIEW_DESC s = {};
        s.Format                         = DXGI_FORMAT_R32_FLOAT;
        s.ViewDimension                  = D3D12_SRV_DIMENSION_TEXTURE2DARRAY;
        s.Shader4ComponentMapping        = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        s.Texture2DArray.MipLevels       = 1;
        s.Texture2DArray.FirstArraySlice = 0;
        s.Texture2DArray.ArraySize       = m_count;
        device->CreateShaderResourceView(m_texture.Get(), &s,
                                         m_srvHeap->GetCPUDescriptorHandleForHeapStart());

        m_state = D3D12_RESOURCE_STATE_COMMON;
        LogInfo("CascadedShadowMap created.");
        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

void CascadedShadowMap::Reset() {
    m_texture.Reset();
    m_dsvHeap.Reset();
    m_srvHeap.Reset();
    m_dsvStride = m_size = m_count = 0;
    m_state = D3D12_RESOURCE_STATE_COMMON;
}

void CascadedShadowMap::BeginCascadePass(ID3D12GraphicsCommandList* cmd) {
    if (m_state != D3D12_RESOURCE_STATE_DEPTH_WRITE) {
        D3D12_RESOURCE_BARRIER b = {};
        b.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        b.Transition.pResource   = m_texture.Get();
        b.Transition.StateBefore = m_state;
        b.Transition.StateAfter  = D3D12_RESOURCE_STATE_DEPTH_WRITE;
        b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        cmd->ResourceBarrier(1, &b);
        m_state = D3D12_RESOURCE_STATE_DEPTH_WRITE;
    }
}

void CascadedShadowMap::BindCascade(ID3D12GraphicsCommandList* cmd, uint32_t i) {
    D3D12_CPU_DESCRIPTOR_HANDLE dsv = m_dsvHeap->GetCPUDescriptorHandleForHeapStart();
    dsv.ptr += static_cast<SIZE_T>(i) * m_dsvStride;
    cmd->ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);
    cmd->OMSetRenderTargets(0, nullptr, FALSE, &dsv);

    D3D12_VIEWPORT vp = { 0.0f, 0.0f, float(m_size), float(m_size), 0.0f, 1.0f };
    D3D12_RECT     sc = { 0, 0, LONG(m_size), LONG(m_size) };
    cmd->RSSetViewports(1, &vp);
    cmd->RSSetScissorRects(1, &sc);
}

void CascadedShadowMap::EndCascadePass(ID3D12GraphicsCommandList* cmd) {
    if (m_state != D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE) {
        D3D12_RESOURCE_BARRIER b = {};
        b.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        b.Transition.pResource   = m_texture.Get();
        b.Transition.StateBefore = m_state;
        b.Transition.StateAfter  = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
        b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        cmd->ResourceBarrier(1, &b);
        m_state = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
    }
}

} // namespace SGE
