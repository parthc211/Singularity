#pragma once
#include "Scene/DemoScene.h"
#include "Math/MathBenchmark.h"

// CPU-side showcase for the hand-written SSE/AVX math library (Phase 1). Runs a
// correctness check on load (SIMD vs scalar vs DirectXMath) and, on a button,
// times scalar vs SIMD vs DirectXMath. No 3D rendering — it's a data panel.
class SimdMathScene : public SGE::DemoScene {
public:
    const char* Name()        const override { return "SIMD Math"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

private:
    SGE::Math::BenchResults m_results;
    bool m_hasBench = false;
};
