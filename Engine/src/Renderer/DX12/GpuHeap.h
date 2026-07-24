#pragma once
#include "DX12Common.h" // ComPtr, ID3D12*, SGE_THROW_IF_FAILED
#include <cstdint>
#include <vector>

namespace SGE {

// A handle to one PLACED resource sub-allocated from a GpuHeap. Owns the
// ID3D12Resource (a ComPtr); the Offset/Size describe the region it occupies
// inside the heap so the allocator can reclaim it on Free.
struct GpuAllocation {
    Microsoft::WRL::ComPtr<ID3D12Resource> Resource;
    uint64_t Offset = 0; // byte offset of this resource within the heap
    uint64_t Size   = 0; // bytes consumed in the heap (64 KB-aligned)

    bool IsValid() const { return Resource != nullptr; }
};

// ---------------------------------------------------------------------------
// GpuHeap — a custom ID3D12Heap with a first-fit, coalescing free-list
// suballocator that hands out PLACED resources. This is the engine's GPU memory
// manager showcase.
//
// Why this exists (the DX12 concept):
//   * CreateCommittedResource makes an *implicit* heap sized to one resource —
//     simple, but every resource pays a separate 64 KB-aligned heap allocation.
//   * CreatePlacedResource lets us reserve ONE big heap and drop many resources
//     into it at offsets we choose. We own the bookkeeping: which byte ranges
//     are free, splitting on allocate, merging neighbours on free.
//
// Granularity: buffers must be placed on D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGN-
// MENT (64 KB) boundaries, so the free list works in 64 KB units. We use a
// buffer-only DEFAULT heap (ALLOW_ONLY_BUFFERS) for portability — on Resource
// Heap Tier 1 a heap may hold only one resource category, and buffers are the
// universally-supported one.
//
// NOTE on lifetime: Free() releases the placed resource and returns its bytes to
// the pool. If that region is immediately reused, the GPU must no longer be
// reading the old resource — callers that bind these to draws must WaitForGPU()
// (or fence) before freeing. The current demo never binds them, so freeing is
// safe immediately.
// ---------------------------------------------------------------------------
class GpuHeap {
public:
    // sizeBytes is rounded up to a 64 KB multiple. heapType is usually DEFAULT
    // (GPU-local); UPLOAD/READBACK also work for buffer suballocation.
    bool Init(ID3D12Device* device, uint64_t sizeBytes,
              D3D12_HEAP_TYPE heapType = D3D12_HEAP_TYPE_DEFAULT);
    void Shutdown();

    // Allocate a buffer placed resource of at least sizeBytes. Returns an invalid
    // GpuAllocation if nothing fits (out of memory or too fragmented).
    GpuAllocation AllocateBuffer(ID3D12Device* device, uint64_t sizeBytes,
                                 D3D12_RESOURCE_STATES initialState = D3D12_RESOURCE_STATE_COMMON);

    // Release a placed resource and merge its region back into the free list.
    void Free(GpuAllocation& alloc);

    // --- introspection for the ImGui visualizer ---
    struct Block {
        uint64_t Offset;
        uint64_t Size;
        bool     Free;
    };
    const std::vector<Block>& Blocks() const { return m_blocks; }

    uint64_t TotalSize()       const { return m_size; }
    uint64_t UsedBytes()       const { return m_used; }
    uint64_t FreeBytes()       const { return m_size - m_used; }
    uint64_t LargestFreeBlock() const;
    uint32_t AllocationCount() const { return m_allocCount; }

private:
    Microsoft::WRL::ComPtr<ID3D12Heap> m_heap;
    // Ordered, contiguous, non-overlapping blocks that exactly tile [0, m_size).
    std::vector<Block> m_blocks;
    uint64_t m_size       = 0;
    uint64_t m_used       = 0;
    uint32_t m_allocCount = 0;
};

} // namespace SGE
