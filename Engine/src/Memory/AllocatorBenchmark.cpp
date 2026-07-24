#include "Memory/AllocatorBenchmark.h"
#include "Memory/ArenaAllocator.h"
#include "Memory/StackAllocator.h"
#include "Memory/PoolAllocator.h"
#include "Memory/FreeListAllocator.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <vector>
#include <cstdint>

namespace SGE::Mem {
namespace {

using clock_t_ = std::chrono::high_resolution_clock;
volatile float g_sink = 0.0f; // defeat dead-code elimination

template <class F>
double TimeMs(F&& f, int reps) {
    double best = 1e30;
    for (int r = 0; r < reps; ++r) {
        auto t0 = clock_t_::now();
        f();
        auto t1 = clock_t_::now();
        best = std::min(best, std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    return best;
}

bool Aligned(const void* p, std::size_t a) {
    return (reinterpret_cast<std::uintptr_t>(p) & (a - 1)) == 0;
}

// True if the [ptr,ptr+size) ranges are all disjoint.
bool NoOverlap(std::vector<std::pair<std::uintptr_t, std::size_t>> r) {
    std::sort(r.begin(), r.end());
    for (std::size_t i = 1; i < r.size(); ++i)
        if (r[i - 1].first + r[i - 1].second > r[i].first)
            return false;
    return true;
}

bool TestArena() {
    ArenaAllocator a; a.Init(1u << 20);
    std::vector<std::pair<std::uintptr_t, std::size_t>> ranges;
    for (int i = 0; i < 200; ++i) {
        std::size_t sz = 16 + (i % 8) * 16;
        void* p = a.Allocate(sz, 16);
        if (!p || !Aligned(p, 16)) return false;
        ranges.push_back({ reinterpret_cast<std::uintptr_t>(p), sz });
    }
    if (!NoOverlap(ranges)) return false;
    const std::uintptr_t first = ranges.front().first;
    a.Reset();
    if (a.Used() != 0) return false;
    void* p = a.Allocate(16, 16);          // after reset, reuse the start
    return reinterpret_cast<std::uintptr_t>(p) == first;
}

bool TestStack() {
    StackAllocator s; s.Init(1u << 20);
    void* p1 = s.Allocate(100, 16);
    auto  m  = s.GetMarker();
    void* p2 = s.Allocate(200, 16);
    if (!p1 || !p2 || p2 <= p1) return false;
    s.FreeToMarker(m);                      // release p2 (LIFO)
    void* p3 = s.Allocate(50, 16);
    return p3 == p2;                        // p3 reuses p2's freed space
}

bool TestPool() {
    PoolAllocator pool; pool.Init(64, 128, 16);
    std::vector<void*> ptrs;
    for (int i = 0; i < 128; ++i) {
        void* p = pool.Allocate();
        if (!p || !Aligned(p, 16)) return false;
        ptrs.push_back(p);
    }
    if (pool.Allocate() != nullptr) return false; // exhausted
    std::vector<void*> sorted = ptrs;
    std::sort(sorted.begin(), sorted.end());
    if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end()) return false; // all distinct
    pool.Free(ptrs[20]);
    pool.Free(ptrs[10]);
    void* a = pool.Allocate();              // LIFO: last freed (10) comes back first
    void* b = pool.Allocate();
    return a == ptrs[10] && b == ptrs[20];
}

bool TestFreeList() {
    FreeListAllocator fl; fl.Init(1u << 20);

    auto tiles = [&] {
        std::vector<FreeListAllocator::BlockInfo> bs; fl.GetBlocks(bs);
        std::size_t sum = 0; for (auto& b : bs) sum += b.size;
        return sum == fl.Capacity();        // blocks must always tile the buffer
    };
    if (!tiles()) return false;

    std::vector<void*> ptrs;
    for (int i = 0; i < 200; ++i) {
        void* p = fl.Allocate(100);
        if (!p || !Aligned(p, 16)) return false;
        std::memset(p, i & 0xFF, 100);      // stamp a per-block pattern
        ptrs.push_back(p);
    }
    if (!tiles()) return false;
    // If any blocks overlapped, an earlier stamp would be corrupted.
    for (std::size_t i = 0; i < ptrs.size(); ++i)
        if (static_cast<std::uint8_t*>(ptrs[i])[0] != static_cast<std::uint8_t>(i & 0xFF))
            return false;

    for (std::size_t i = 0; i < ptrs.size(); i += 2) fl.Free(ptrs[i]);
    if (!tiles()) return false;
    for (std::size_t i = 1; i < ptrs.size(); i += 2) fl.Free(ptrs[i]);
    if (!tiles()) return false;

    // Everything freed -> coalesce back to one free block, zero used.
    std::vector<FreeListAllocator::BlockInfo> bs; fl.GetBlocks(bs);
    return bs.size() == 1 && bs[0].free && fl.UsedBytes() == 0;
}

} // namespace

AllocResults Run(bool includeTiming) {
    AllocResults res;
    res.correctness.push_back({ "Arena (bump + reset)",     TestArena() });
    res.correctness.push_back({ "Stack (LIFO markers)",     TestStack() });
    res.correctness.push_back({ "Pool (fixed-size O(1))",   TestPool() });
    res.correctness.push_back({ "FreeList (split+coalesce)",TestFreeList() });
    res.allCorrect = true;
    for (auto& c : res.correctness) res.allCorrect &= c.passed;

    if (!includeTiming) return res;

    constexpr int kReps = 16;

    // Arena: N bump allocs (+ one bulk reset) vs N malloc + N free.
    {
        const int N = 100000;
        ArenaAllocator a; a.Init(static_cast<std::size_t>(N) * 64 + 4096);
        std::vector<void*> ptrs(N);
        double cust = TimeMs([&]{
            a.Reset();
            float acc = 0; for (int i = 0; i < N; ++i) acc += (a.Allocate(48, 16) != nullptr);
            g_sink = acc;
        }, kReps);
        double mal = TimeMs([&]{
            for (int i = 0; i < N; ++i) ptrs[i] = std::malloc(48);
            for (int i = 0; i < N; ++i) std::free(ptrs[i]);
            g_sink = static_cast<float>(N);
        }, kReps);
        res.bench.push_back({ "Arena: 100k alloc (+reset) vs malloc/free", mal, cust });
    }

    // Pool: allocate all blocks then free all, vs malloc/free of same-size objects.
    {
        const int N = 100000;
        PoolAllocator pool; pool.Init(64, N, 16);
        std::vector<void*> ptrs(N);
        double cust = TimeMs([&]{
            for (int i = 0; i < N; ++i) ptrs[i] = pool.Allocate();
            for (int i = 0; i < N; ++i) pool.Free(ptrs[i]);
            g_sink = 1.0f;
        }, kReps);
        double mal = TimeMs([&]{
            for (int i = 0; i < N; ++i) ptrs[i] = std::malloc(64);
            for (int i = 0; i < N; ++i) std::free(ptrs[i]);
            g_sink = 2.0f;
        }, kReps);
        res.bench.push_back({ "Pool: 100k alloc+free vs malloc/free", mal, cust });
    }

    // FreeList: allocate a working set then free it, vs malloc. (First-fit is
    // O(live), so we keep the set modest.)
    {
        const int K = 1500;
        FreeListAllocator fl; fl.Init(4u << 20);
        std::vector<void*> ptrs(K);
        double cust = TimeMs([&]{
            for (int i = 0; i < K; ++i) ptrs[i] = fl.Allocate(64 + (i & 63));
            for (int i = 0; i < K; ++i) fl.Free(ptrs[i]);
            g_sink = 3.0f;
        }, kReps);
        double mal = TimeMs([&]{
            for (int i = 0; i < K; ++i) ptrs[i] = std::malloc(64 + (i & 63));
            for (int i = 0; i < K; ++i) std::free(ptrs[i]);
            g_sink = 4.0f;
        }, kReps);
        res.bench.push_back({ "FreeList: 1.5k alloc+free vs malloc/free", mal, cust });
    }

    return res;
}

} // namespace SGE::Mem
