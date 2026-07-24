#include "Renderer/DX12/ShadowMap.h"
#include "Core/Logger.h"

namespace SGE {

bool ShadowMap::Create(ID3D12Device* device, uint32_t size) {
    try {
        Reset();
        m_size = size;

        D3D12_HEAP_PROPERTIES heapProps = {};
        heapProps.Type = D3D12_HEAP_TYPE_DEFAULT;

        // Typeless so we can place both a depth view and a float SRV over it.
        D3D12_RESOURCE_DESC desc = {};
        desc.Dimension        = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        desc.Width            = size;
        desc.Height           = size;
        desc.DepthOrArraySize = 1;
        desc.MipLevels        = 1;
        desc.Format           = DXGI_FORMAT_R32_TYPELESS;
        desc.SampleDesc.Count = 1;
        desc.Flags            = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;

        D3D12_CLEAR_VALUE clear = {};
        clear.Format               = DXGI_FORMAT_D32_FLOAT;
        clear.DepthStencil.Depth   = 1.0f;

        SGE_THROW_IF_FAILED(device->CreateCommittedResource(
            &heapProps, D3D12_HEAP_FLAG_NONE, &desc,
            D3D12_RESOURCE_STATE_COMMON, &clear, IID_PPV_ARGS(&m_texture)));

        // DSV (typed as depth).
        D3D12_DESCRIPTOR_HEAP_DESC dsvHeapDesc = {};
        dsvHeapDesc.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
        dsvHeapDesc.NumDescriptors = 1;
        SGE_THROW_IF_FAILED(device->CreateDescriptorHeap(&dsvHeapDesc, IID_PPV_ARGS(&m_dsvHeap)));

        D3D12_DEPTH_STENCIL_VIEW_DESC dsvDesc = {};
        dsvDesc.Format        = DXGI_FORMAT_D32_FLOAT;
        dsvDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
        device->CreateDepthStencilView(m_texture.Get(), &dsvDesc,
                                       m_dsvHeap->GetCPUDescriptorHandleForHeapStart());

        // SRV (typed as float, shader-visible) for sampling in the main pass.
        D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
        srvHeapDesc.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
        srvHeapDesc.NumDescriptors = 1;
        srvHeapDesc.Flags          = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        SGE_THROW_IF_FAILED(device->CreateDescriptorHeap(&srvHeapDesc, IID_PPV_ARGS(&m_srvHeap)));

        D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
        srvDesc.Format                  = DXGI_FORMAT_R32_FLOAT;
        srvDesc.ViewDimension           = D3D12_SRV_DIMENSION_TEXTURE2D;
        srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        srvDesc.Texture2D.MipLevels     = 1;
        device->CreateShaderResourceView(m_texture.Get(), &srvDesc,
                                         m_srvHeap->GetCPUDescriptorHandleForHeapStart());

        m_state = D3D12_RESOURCE_STATE_COMMON;
        LogInfo("ShadowMap created.");
        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

void ShadowMap::Reset() {
    m_texture.Reset();
    m_dsvHeap.Reset();
    m_srvHeap.Reset();
    m_state = D3D12_RESOURCE_STATE_COMMON;
    m_size  = 0;
}

void ShadowMap::BeginShadowPass(ID3D12GraphicsCommandList* cmd) {
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

    D3D12_CPU_DESCRIPTOR_HANDLE dsv = m_dsvHeap->GetCPUDescriptorHandleForHeapStart();
    cmd->ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);
    cmd->OMSetRenderTargets(0, nullptr, FALSE, &dsv); // depth only, no colour

    D3D12_VIEWPORT vp = { 0.0f, 0.0f, float(m_size), float(m_size), 0.0f, 1.0f };
    D3D12_RECT     sc = { 0, 0, LONG(m_size), LONG(m_size) };
    cmd->RSSetViewports(1, &vp);
    cmd->RSSetScissorRects(1, &sc);
}

void ShadowMap::EndShadowPass(ID3D12GraphicsCommandList* cmd) {
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
