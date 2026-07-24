#pragma once

#include "Renderer/DX12/DX12Common.h"
#include "Renderer/DX12/GpuHeap.h"

namespace SGE {

// Index buffer (32-bit R32_UINT) backed by a PLACED resource sub-allocated from a
// shared GpuHeap, mirroring VertexBuffer. See VertexBuffer.h for the rationale.
class IndexBuffer
{
public:
    bool Upload(ID3D12Device* device, GpuHeap& heap, const uint32_t* data, uint32_t count);

    // Returns the heap region (GPU must be idle first — see VertexBuffer::Reset).
    void Reset();

    const D3D12_INDEX_BUFFER_VIEW& GetView()  const { return m_view;  }
    uint32_t                       Count()    const { return m_count; }

private:
    GpuHeap*                m_heap = nullptr;
    GpuAllocation           m_alloc;
    D3D12_INDEX_BUFFER_VIEW m_view  = {};
    uint32_t                m_count = 0;
};

} // namespace SGE
