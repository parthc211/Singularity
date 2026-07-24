#include "Renderer/DX12/ConstantBuffer.h"
#include "Core/Logger.h"

#include <cstring>

namespace SGE {

static constexpr uint32_t Align256(uint32_t n)
{
    return (n + 255u) & ~255u;
}

bool ConstantBuffer::Create(ID3D12Device* device, uint32_t dataSizeBytes)
{
    m_alignedSize = Align256(dataSizeBytes);

    D3D12_HEAP_PROPERTIES heap = {};
    heap.Type = D3D12_HEAP_TYPE_UPLOAD;

    D3D12_RESOURCE_DESC desc = {};
    desc.Dimension          = D3D12_RESOURCE_DIMENSION_BUFFER;
    desc.Width              = uint64_t(m_alignedSize) * FrameCount;
    desc.Height             = 1;
    desc.DepthOrArraySize   = 1;
    desc.MipLevels          = 1;
    desc.Format             = DXGI_FORMAT_UNKNOWN;
    desc.SampleDesc.Count   = 1;
    desc.Layout             = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    try {
        SGE_THROW_IF_FAILED(device->CreateCommittedResource(
            &heap, D3D12_HEAP_FLAG_NONE, &desc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr, IID_PPV_ARGS(&m_buffer)));

        D3D12_RANGE noRead = {};
        SGE_THROW_IF_FAILED(m_buffer->Map(0, &noRead, reinterpret_cast<void**>(&m_mapped)));
        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

void ConstantBuffer::Update(uint32_t frameIndex, const void* data, uint32_t sizeBytes)
{
    std::memcpy(m_mapped + frameIndex * m_alignedSize, data, sizeBytes);
}

D3D12_GPU_VIRTUAL_ADDRESS ConstantBuffer::GetGPUAddress(uint32_t frameIndex) const
{
    return m_buffer->GetGPUVirtualAddress() + frameIndex * m_alignedSize;
}

void ConstantBuffer::Reset()
{
    if (m_buffer) {
        m_buffer->Unmap(0, nullptr);
        m_mapped = nullptr;
    }
    m_buffer.Reset();
    m_alignedSize = 0;
}

} // namespace SGE
