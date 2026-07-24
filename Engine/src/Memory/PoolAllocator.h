#pragma once
#include "Memory/MemoryCommon.h"
#include <vector>

namespace SGE::Mem {

// ---------------------------------------------------------------------------
// PoolAllocator
//
// Hands out fixed-size blocks in O(1) with zero fragmentation (every block is
// the same size, so any free block fits any request). The free list is
// *intrusive*: each free block stores the pointer to the next free block inside
// its own memory, so the pool needs no side tables. Allocate() pops the head;
// Free() pushes onto the head. Ideal for many same-type objects (particles,
// ECS components, nodes).
// ---------------------------------------------------------------------------
class PoolAllocator {
public:
    void Init(std::size_t blockSize, std::size_t blockCount, std::size_t align = 16) {
        Shutdown();
        // A block must be at least large enough to hold the free-list pointer.
        m_blockSize  = AlignUp(blockSize < sizeof(void*) ? sizeof(void*) : blockSize, align);
        m_blockCount = blockCount;
        m_align      = align;
        m_base       = static_cast<std::uint8_t*>(BackingAlloc(m_blockSize * blockCount, align));

        // Thread a free list through every block (build in reverse so the head
        // ends up at block 0 — nicer for the visualizer).
        m_freeHead = nullptr;
        for (std::size_t i = blockCount; i-- > 0; ) {
            void* block = m_base + i * m_blockSize;
            *reinterpret_cast<void**>(block) = m_freeHead;
            m_freeHead = block;
        }
        m_freeCount = blockCount;
    }
    void Shutdown() {
        if (m_base) BackingFree(m_base, m_align);
        m_base = nullptr;
        m_freeHead = nullptr;
        m_blockCount = m_freeCount = 0;
    }
    ~PoolAllocator() { Shutdown(); }

    void* Allocate() {
        if (!m_freeHead) return nullptr;          // pool exhausted
        void* block = m_freeHead;
        m_freeHead  = *reinterpret_cast<void**>(block); // pop: next free becomes head
        --m_freeCount;
        return block;
    }
    void Free(void* p) {
        if (!p) return;
        *reinterpret_cast<void**>(p) = m_freeHead; // push onto the free list
        m_freeHead = p;
        ++m_freeCount;
    }

    std::size_t BlockSize()  const { return m_blockSize; }
    std::size_t BlockCount() const { return m_blockCount; }
    std::size_t FreeCount()  const { return m_freeCount; }
    std::size_t UsedCount()  const { return m_blockCount - m_freeCount; }

    // For the visualizer: out[i] = true if block i is currently allocated.
    // Derived by walking the free list (no redundant bookkeeping in the pool).
    void GetSlotUsage(std::vector<std::uint8_t>& out) const {
        out.assign(m_blockCount, 1); // assume used...
        for (void* b = m_freeHead; b; b = *reinterpret_cast<void**>(b)) {
            const std::size_t idx = (static_cast<std::uint8_t*>(b) - m_base) / m_blockSize;
            out[idx] = 0; // ...then clear the ones on the free list
        }
    }

private:
    std::uint8_t* m_base       = nullptr;
    void*         m_freeHead   = nullptr;
    std::size_t   m_blockSize  = 0;
    std::size_t   m_blockCount = 0;
    std::size_t   m_freeCount  = 0;
    std::size_t   m_align      = 16;
};

} // namespace SGE::Mem
