#pragma once
#include "Memory/MemoryCommon.h"

namespace SGE::Mem {

// ---------------------------------------------------------------------------
// ArenaAllocator (a.k.a. linear / bump allocator)
//
// The simplest and fastest allocator: keep a single cursor into a buffer and
// advance ("bump") it on each allocation. There is NO per-object free — you
// reclaim everything at once with Reset(). Perfect for per-frame scratch memory
// or load-time data with a clear bulk lifetime. Allocation is a few adds and a
// compare; this is the CPU cousin of the GPU DynamicUploadBuffer arena.
// ---------------------------------------------------------------------------
class ArenaAllocator {
public:
    void Init(std::size_t bytes, std::size_t backingAlign = 64) {
        Shutdown();
        m_base    = static_cast<std::uint8_t*>(BackingAlloc(bytes, backingAlign));
        m_align   = backingAlign;
        m_capacity = bytes;
        m_offset  = 0;
    }
    void Shutdown() {
        if (m_base) BackingFree(m_base, m_align);
        m_base = nullptr;
        m_capacity = m_offset = 0;
    }
    ~ArenaAllocator() { Shutdown(); }

    void* Allocate(std::size_t size, std::size_t align = 16) {
        const std::size_t aligned = AlignUp(m_offset, align);
        if (aligned + size > m_capacity)
            return nullptr; // out of arena space
        m_offset = aligned + size;
        return m_base + aligned;
    }

    // Reclaim everything at once — O(1), no per-object bookkeeping.
    void Reset() { m_offset = 0; }

    std::size_t Used()     const { return m_offset; }
    std::size_t Capacity() const { return m_capacity; }

private:
    std::uint8_t* m_base     = nullptr;
    std::size_t   m_align    = 64;
    std::size_t   m_capacity = 0;
    std::size_t   m_offset   = 0;
};

} // namespace SGE::Mem
