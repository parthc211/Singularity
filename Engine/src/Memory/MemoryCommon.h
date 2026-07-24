#pragma once
#include <cstddef>
#include <cstdint>
#include <new>

// Shared helpers for the hand-written CPU allocators (Phase 2). Each allocator
// owns one big backing buffer (an over-aligned malloc) and hands out pieces of
// it, so we never call the OS allocator on the hot path.
namespace SGE::Mem {

// Round v up to the next multiple of a power-of-two alignment a.
inline std::size_t AlignUp(std::size_t v, std::size_t a) {
    return (v + (a - 1)) & ~(a - 1);
}

// Over-aligned backing storage (so payloads can satisfy SIMD/cache alignment).
inline void* BackingAlloc(std::size_t bytes, std::size_t align) {
    return ::operator new(bytes, std::align_val_t(align));
}
inline void BackingFree(void* p, std::size_t align) {
    ::operator delete(p, std::align_val_t(align));
}

} // namespace SGE::Mem
