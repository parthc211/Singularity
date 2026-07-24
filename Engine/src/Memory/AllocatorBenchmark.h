#pragma once
#include <vector>

namespace SGE::Mem {

struct AllocCorrectness {
    const char* name;
    bool        passed;
};

// One timing row: best-of-N ms for the same workload via malloc/free vs the
// custom allocator.
struct AllocBench {
    const char* name;
    double      mallocMs;
    double      customMs;
};

struct AllocResults {
    std::vector<AllocCorrectness> correctness;
    std::vector<AllocBench>       bench;     // empty unless timing requested
    bool allCorrect = false;
};

// Validate the allocators (alignment, non-overlap, LIFO release, coalescing,
// buffer tiling invariants). If includeTiming, also benchmark vs malloc.
AllocResults Run(bool includeTiming);

} // namespace SGE::Mem
