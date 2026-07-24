#include "Renderer/DX12/DepthBuffer.h"
#include "Core/Logger.h"

namespace SGE {

bool DepthBuffer::Create(ID3D12Device* device, uint32_t width, uint32_t height)
{
    try {
        if (!m_dsvHeap) {
            D3D12_DESCRIPTOR_HEAP_DESC heapDesc = {};
            heapDesc.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
            heapDesc.NumDescriptors = 1;
            SGE_THROW_IF_FAILED(device->CreateDescriptorHeap(&heapDesc, IID_PPV_ARGS(&m_dsvHeap)));
        }

        D3D12_HEAP_PROPERTIES heap = {};
        heap.Type = D3D12_HEAP_TYPE_DEFAULT;

        D3D12_RESOURCE_DESC desc = {};
        desc.Dimension        = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        desc.Width            = width;
        desc.Height           = height;
        desc.DepthOrArraySize = 1;
        desc.MipLevels        = 1;
        desc.Format           = DXGI_FORMAT_D32_FLOAT;
        desc.SampleDesc.Count = 1;
        desc.Flags            = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;

        D3D12_CLEAR_VALUE clearVal          = {};
        clearVal.Format                     = DXGI_FORMAT_D32_FLOAT;
        clearVal.DepthStencil.Depth         = 1.0f;

        SGE_THROW_IF_FAILED(device->CreateCommittedResource(
            &heap, D3D12_HEAP_FLAG_NONE, &desc,
            D3D12_RESOURCE_STATE_DEPTH_WRITE,
            &clearVal, IID_PPV_ARGS(&m_resource)));

        device->CreateDepthStencilView(
            m_resource.Get(), nullptr,
            m_dsvHeap->GetCPUDescriptorHandleForHeapStart());

        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

void DepthBuffer::Resize(ID3D12Device* device, uint32_t width, uint32_t height)
{
    m_resource.Reset();
    Create(device, width, height);
}

D3D12_CPU_DESCRIPTOR_HANDLE DepthBuffer::GetDSV() const
{
    return m_dsvHeap->GetCPUDescriptorHandleForHeapStart();
}

} // namespace SGE
