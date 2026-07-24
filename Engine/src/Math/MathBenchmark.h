#pragma once
#include <vector>

namespace SGE::Math {

// One correctness check: does the SIMD op match the scalar + DirectXMath
// reference within epsilon?
struct CorrectnessEntry {
    const char* name;
    bool        passed;
    float       maxError;
};

// One timing row: best-of-N milliseconds for the same workload done three ways.
// dxmathMs / scalarMs may be < 0 to mean "not measured for this row".
struct BenchEntry {
    const char* name;
    double      scalarMs;
    double      simdMs;
    double      dxmathMs;
};

struct BenchResults {
    std::vector<CorrectnessEntry> correctness;
    std::vector<BenchEntry>       bench;     // empty unless timing was requested
    bool allCorrect = false;
    bool avxUsed    = false;                 // false if the CPU lacks AVX (SoA row skipped)
};

// Validate the SIMD math against scalar + DirectXMath. If includeTiming, also run
// the (slower) micro-benchmarks. Pure CPU; safe to call from anywhere.
BenchResults Run(bool includeTiming);

} // namespace SGE::Math
