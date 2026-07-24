#pragma once
#include "Scene/DemoScene.h"
#include "Memory/ArenaAllocator.h"
#include "Memory/StackAllocator.h"
#include "Memory/PoolAllocator.h"
#include "Memory/FreeListAllocator.h"
#include "Memory/AllocatorBenchmark.h"

#include <vector>
#include <random>

// CPU-side showcase for the hand-written allocators (Phase 2). Tabs let you
// drive each allocator interactively and watch its memory map; a benchmark tab
// times them against malloc. No 3D rendering — a data panel.
class AllocatorScene : public SGE::DemoScene {
public:
    const char* Name()        const override { return "CPU Allocators"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnImGui() override;

private:
    void ArenaTab();
    void StackTab();
    void PoolTab();
    void FreeListTab();
    void BenchTab();
    void DrawPoolGrid();
    void DrawFreeListMap();

    SGE::Mem::ArenaAllocator    m_arena;
    SGE::Mem::StackAllocator    m_stack;
    SGE::Mem::PoolAllocator     m_pool;
    SGE::Mem::FreeListAllocator m_freelist;

    std::vector<SGE::Mem::StackAllocator::Marker> m_markers;
    std::vector<void*> m_poolPtrs;
    std::vector<void*> m_flPtrs;

    SGE::Mem::AllocResults m_results;
    bool m_hasBench = false;

    std::mt19937 m_rng{ 0xA11C };
};
