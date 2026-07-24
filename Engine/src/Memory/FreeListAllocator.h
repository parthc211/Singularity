#pragma once
#include "Memory/MemoryCommon.h"
#include <vector>

namespace SGE::Mem {

// ---------------------------------------------------------------------------
// FreeListAllocator
//
// General-purpose, variable-size allocator over one buffer — the CPU sibling of
// the GPU GpuHeap. The buffer is tiled by a doubly-linked list of blocks; each
// block carries an *intrusive* header (its size, a pointer to the previous block
// for O(1) coalescing, and a free flag). Allocate does first-fit + split; Free
// marks the block free and coalesces with adjacent free neighbours so the heap
// doesn't stay fragmented. Supports arbitrary alloc/free order — at the cost of
// the fragmentation the visualizer makes visible.
// ---------------------------------------------------------------------------
class FreeListAllocator {
public:
    void Init(std::size_t bytes);
    void Shutdown();
    ~FreeListAllocator() { Shutdown(); }

    // Returns a 16-byte-aligned pointer, or nullptr if nothing fits.
    void* Allocate(std::size_t size);
    void  Free(void* ptr);

    // --- introspection for the visualizer ---
    struct BlockInfo { std::size_t offset; std::size_t size; bool free; };
    void GetBlocks(std::vector<BlockInfo>& out) const;

    std::size_t Capacity()        const { return m_capacity; }
    std::size_t UsedBytes()       const { return m_used; }
    std::size_t FreeBytes()       const { return m_capacity - m_used; }
    std::size_t LargestFreeBlock() const;
    std::size_t AllocationCount()  const { return m_allocCount; }

private:
    // 16-byte aligned so payloads (which follow the header) stay 16-aligned.
    struct alignas(16) BlockHeader {
        std::size_t  size;  // total block size, header included
        BlockHeader* prev;  // previous block in address order (for coalescing)
        bool         free;
    };
    static constexpr std::size_t kHeaderSize = sizeof(BlockHeader); // multiple of 16

    BlockHeader* Next(BlockHeader* b) const;
    bool         InBuffer(BlockHeader* b) const;

    std::uint8_t* m_base       = nullptr;
    std::size_t   m_capacity   = 0;
    std::size_t   m_used       = 0; // bytes in allocated blocks (header + payload)
    std::size_t   m_allocCount = 0;
    BlockHeader*  m_first      = nullptr;
};

} // namespace SGE::Mem
