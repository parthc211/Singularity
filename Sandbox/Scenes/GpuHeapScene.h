#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/GpuHeap.h"

#include <vector>
#include <random>

// Interactive showcase for the custom GPU heap allocator. Reserves one
// ID3D12Heap and lets you allocate/free placed buffer resources from it via
// ImGui buttons, with a live visualization of used/free regions and
// fragmentation stats. No 3D geometry — the scene is pure data viz, which is
// itself a nice demonstration that scenes need not render the world.
class GpuHeapScene : public SGE::DemoScene {
public:
    const char* Name()        const override { return "GPU Heap Allocator"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnImGui() override;

private:
    void DoAlloc(uint64_t bytes);
    void AllocRandom();
    void FreeRandom();
    void FreeAll();
    void DrawHeapBar();

    SGE::GpuHeap                     m_heap;
    std::vector<SGE::GpuAllocation>  m_allocs;
    ID3D12Device*                    m_device = nullptr; // cached for button-driven allocs
    std::mt19937                     m_rng{ 0xC0FFEEu };
    bool                             m_loaded = false;
};
