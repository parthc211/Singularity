#include "Memory/FreeListAllocator.h"

namespace SGE::Mem {

FreeListAllocator::BlockHeader* FreeListAllocator::Next(BlockHeader* b) const {
    return reinterpret_cast<BlockHeader*>(reinterpret_cast<std::uint8_t*>(b) + b->size);
}

bool FreeListAllocator::InBuffer(BlockHeader* b) const {
    auto* p = reinterpret_cast<std::uint8_t*>(b);
    return p >= m_base && p < m_base + m_capacity;
}

void FreeListAllocator::Init(std::size_t bytes) {
    Shutdown();
    m_capacity = bytes & ~std::size_t(15); // keep the buffer a multiple of 16
    m_base     = static_cast<std::uint8_t*>(BackingAlloc(m_capacity, 64));
    m_used     = 0;
    m_allocCount = 0;

    // One big free block spanning the whole buffer.
    m_first = reinterpret_cast<BlockHeader*>(m_base);
    m_first->size = m_capacity;
    m_first->prev = nullptr;
    m_first->free = true;
}

void FreeListAllocator::Shutdown() {
    if (m_base) BackingFree(m_base, 64);
    m_base = nullptr;
    m_first = nullptr;
    m_capacity = m_used = m_allocCount = 0;
}

void* FreeListAllocator::Allocate(std::size_t size) {
    if (size == 0 || !m_base) return nullptr;

    const std::size_t need = AlignUp(kHeaderSize + size, 16);

    // First-fit: walk the address-ordered block list.
    for (BlockHeader* b = m_first; InBuffer(b); b = Next(b)) {
        if (!b->free || b->size < need)
            continue;

        // Split off the remainder as a new free block if it's big enough to be
        // useful (must hold at least a header + a minimal payload).
        if (b->size - need >= kHeaderSize + 16) {
            BlockHeader* nb = reinterpret_cast<BlockHeader*>(reinterpret_cast<std::uint8_t*>(b) + need);
            nb->size = b->size - need;
            nb->prev = b;
            nb->free = true;
            b->size  = need;
            BlockHeader* after = Next(nb);
            if (InBuffer(after)) after->prev = nb;
        }

        b->free = false;
        m_used += b->size;
        ++m_allocCount;
        return reinterpret_cast<std::uint8_t*>(b) + kHeaderSize;
    }
    return nullptr; // no fit — full or too fragmented
}

void FreeListAllocator::Free(void* ptr) {
    if (!ptr) return;
    BlockHeader* b = reinterpret_cast<BlockHeader*>(static_cast<std::uint8_t*>(ptr) - kHeaderSize);
    if (b->free) return; // guard against double free

    b->free = true;
    m_used -= b->size;
    --m_allocCount;

    // Merge with the following block if it's free.
    BlockHeader* nx = Next(b);
    if (InBuffer(nx) && nx->free) {
        b->size += nx->size;
        BlockHeader* after = Next(b);
        if (InBuffer(after)) after->prev = b;
    }
    // Merge into the previous block if it's free.
    if (b->prev && b->prev->free) {
        BlockHeader* p = b->prev;
        p->size += b->size;
        BlockHeader* after = Next(p);
        if (InBuffer(after)) after->prev = p;
        b = p;
    }
}

void FreeListAllocator::GetBlocks(std::vector<BlockInfo>& out) const {
    out.clear();
    for (BlockHeader* b = m_first; InBuffer(b); b = Next(b))
        out.push_back({ static_cast<std::size_t>(reinterpret_cast<std::uint8_t*>(b) - m_base),
                        b->size, b->free });
}

std::size_t FreeListAllocator::LargestFreeBlock() const {
    std::size_t largest = 0;
    for (BlockHeader* b = m_first; InBuffer(b); b = Next(b))
        if (b->free && b->size > largest)
            largest = b->size;
    return largest;
}

} // namespace SGE::Mem
