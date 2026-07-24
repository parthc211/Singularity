#pragma once

#include "Renderer/DX12/DX12Common.h"
#include "Renderer/DX12/GpuHeap.h"

namespace SGE {

// Vertex buffer backed by a PLACED resource sub-allocated from a shared GpuHeap
// (an UPLOAD-type heap for static geometry). CPU writes the vertices once via
// Map; the GPU reads directly from upload memory — fine for static meshes.
//
// This is the Phase-5 follow-up: instead of CreateCommittedResource (one implicit
// heap per buffer), the bytes come from one big engine-owned heap, managed by the
// GpuHeap free-list allocator.
class VertexBuffer
{
public:
    bool Upload(ID3D12Device* device, GpuHeap& heap,
                const void* data, uint32_t sizeBytes, uint32_t strideBytes);

    // Returns the heap region to the allocator. The owner must have made the GPU
    // idle first (the freed bytes may be reused by a later allocation).
    void Reset();

    const D3D12_VERTEX_BUFFER_VIEW& GetView() const { return m_view; }

private:
    GpuHeap*                 m_heap = nullptr; // heap this allocation came from
    GpuAllocation            m_alloc;          // owns the placed ID3D12Resource
    D3D12_VERTEX_BUFFER_VIEW m_view = {};
};

} // namespace SGE
