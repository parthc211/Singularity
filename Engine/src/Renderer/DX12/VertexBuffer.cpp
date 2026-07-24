#include "Renderer/DX12/VertexBuffer.h"
#include "Core/Logger.h"

#include <cstring>

namespace SGE {

bool VertexBuffer::Upload(ID3D12Device* device, GpuHeap& heap,
                          const void* data, uint32_t sizeBytes, uint32_t strideBytes)
{
    try {
        if (m_heap) { m_heap->Free(m_alloc); m_heap = nullptr; } // guard re-upload

        // Placed resources in an UPLOAD heap must be created in (and stay in)
        // GENERIC_READ — same rule as the old committed upload buffer.
        m_alloc = heap.AllocateBuffer(device, sizeBytes, D3D12_RESOURCE_STATE_GENERIC_READ);
        if (!m_alloc.IsValid())
            throw std::runtime_error("VertexBuffer: GpuHeap allocation failed (heap full/fragmented?)");
        m_heap = &heap;

        // UPLOAD memory is CPU-writable: map, copy the vertices, unmap.
        void* mapped = nullptr;
        D3D12_RANGE noRead = { 0, 0 }; // we won't read back on the CPU
        SGE_THROW_IF_FAILED(m_alloc.Resource->Map(0, &noRead, &mapped));
        std::memcpy(mapped, data, sizeBytes);
        m_alloc.Resource->Unmap(0, nullptr);

        // GetGPUVirtualAddress on a placed resource returns the address at its
        // offset within the heap — exactly what the vertex-buffer view needs.
        m_view.BufferLocation = m_alloc.Resource->GetGPUVirtualAddress();
        m_view.SizeInBytes    = sizeBytes;
        m_view.StrideInBytes  = strideBytes;

        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

void VertexBuffer::Reset()
{
    if (m_heap) { m_heap->Free(m_alloc); m_heap = nullptr; }
    m_view = {};
}

} // namespace SGE
