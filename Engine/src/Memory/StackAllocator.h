#pragma once
#include "Memory/MemoryCommon.h"

namespace SGE::Mem {

// ---------------------------------------------------------------------------
// StackAllocator
//
// A bump allocator that also supports freeing in LIFO order via "markers". Take
// a marker (the current cursor), allocate some scratch, then FreeToMarker() to
// roll the cursor back — releasing everything allocated since, all at once. Ideal
// for nested/temporary scopes (e.g. build a mesh, use it, unwind). Same O(1)
// allocation as the arena, plus cheap scoped release.
// ---------------------------------------------------------------------------
class StackAllocator {
public:
    using Marker = std::size_t;

    void Init(std::size_t bytes, std::size_t backingAlign = 64) {
        Shutdown();
        m_base     = static_cast<std::uint8_t*>(BackingAlloc(bytes, backingAlign));
        m_align    = backingAlign;
        m_capacity = bytes;
        m_offset   = 0;
    }
    void Shutdown() {
        if (m_base) BackingFree(m_base, m_align);
        m_base = nullptr;
        m_capacity = m_offset = 0;
    }
    ~StackAllocator() { Shutdown(); }

    void* Allocate(std::size_t size, std::size_t align = 16) {
        const std::size_t aligned = AlignUp(m_offset, align);
        if (aligned + size > m_capacity)
            return nullptr;
        m_offset = aligned + size;
        return m_base + aligned;
    }

    Marker GetMarker() const          { return m_offset; }
    void   FreeToMarker(Marker m)     { if (m <= m_offset) m_offset = m; } // LIFO release
    void   Reset()                    { m_offset = 0; }

    std::size_t Used()     const { return m_offset; }
    std::size_t Capacity() const { return m_capacity; }

private:
    std::uint8_t* m_base     = nullptr;
    std::size_t   m_align    = 64;
    std::size_t   m_capacity = 0;
    std::size_t   m_offset   = 0;
};

} // namespace SGE::Mem
