#include "Renderer/DX12/IndexBuffer.h"
#include "Core/Logger.h"

#include <cstring>

namespace SGE {

bool IndexBuffer::Upload(ID3D12Device* device, GpuHeap& heap, const uint32_t* data, uint32_t count)
{
    const uint32_t sizeBytes = count * sizeof(uint32_t);

    try {
        if (m_heap) { m_heap->Free(m_alloc); m_heap = nullptr; } // guard re-upload

        m_alloc = heap.AllocateBuffer(device, sizeBytes, D3D12_RESOURCE_STATE_GENERIC_READ);
        if (!m_alloc.IsValid())
            throw std::runtime_error("IndexBuffer: GpuHeap allocation failed (heap full/fragmented?)");
        m_heap = &heap;

        void* mapped = nullptr;
        D3D12_RANGE noRead = { 0, 0 };
        SGE_THROW_IF_FAILED(m_alloc.Resource->Map(0, &noRead, &mapped));
        std::memcpy(mapped, data, sizeBytes);
        m_alloc.Resource->Unmap(0, nullptr);

        m_view.BufferLocation = m_alloc.Resource->GetGPUVirtualAddress();
        m_view.SizeInBytes    = sizeBytes;
        m_view.Format         = DXGI_FORMAT_R32_UINT;
        m_count               = count;

        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

void IndexBuffer::Reset()
{
    if (m_heap) { m_heap->Free(m_alloc); m_heap = nullptr; }
    m_view  = {};
    m_count = 0;
}

} // namespace SGE
