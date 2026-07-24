#include "Scenes/AllocatorScene.h"
#include "Core/Logger.h"
#include "imgui.h"
#include <cmath>
#include <string>

using namespace SGE;

namespace {
constexpr std::size_t kArenaCap = 256 * 1024;
constexpr std::size_t kStackCap = 256 * 1024;
constexpr std::size_t kPoolBlock = 64;
constexpr std::size_t kPoolCount = 512;
constexpr std::size_t kFreeListCap = 256 * 1024;

void UsageBar(std::size_t used, std::size_t cap) {
    const float frac = cap ? static_cast<float>(static_cast<double>(used) / static_cast<double>(cap)) : 0.0f;
    char label[64];
    std::snprintf(label, sizeof(label), "%.1f / %.1f KB", used / 1024.0, cap / 1024.0);
    ImGui::ProgressBar(frac, ImVec2(-1.0f, 0.0f), label);
}
}

const char* AllocatorScene::Description() const {
    return "Hand-written CPU allocators, each carving one pre-allocated buffer: "
           "Arena (bump), Stack (LIFO markers), Pool (fixed-size O(1)), and "
           "FreeList (variable-size, split + coalesce). Drive them and watch the "
           "memory map; the Benchmark tab times them against malloc.";
}

void AllocatorScene::OnLoad(const DemoContext&) {
    m_arena.Init(kArenaCap);
    m_stack.Init(kStackCap);
    m_pool.Init(kPoolBlock, kPoolCount, 16);
    m_freelist.Init(kFreeListCap);
    m_markers.clear();
    m_poolPtrs.clear();
    m_flPtrs.clear();
    m_results  = Mem::Run(false); // correctness on load
    m_hasBench = false;

    for (const auto& c : m_results.correctness)
        if (!c.passed) LogError(std::string("Allocator FAIL: ") + c.name);
    LogInfo(m_results.allCorrect ? "CPU allocators: all correctness checks pass."
                                 : "CPU allocators: correctness FAILED (see above).");
}

void AllocatorScene::OnUnload() {
    m_arena.Shutdown();
    m_stack.Shutdown();
    m_pool.Shutdown();
    m_freelist.Shutdown();
    m_markers.clear();
    m_poolPtrs.clear();
    m_flPtrs.clear();
}

void AllocatorScene::OnImGui() {
    if (ImGui::BeginTabBar("allocators")) {
        if (ImGui::BeginTabItem("Arena"))     { ArenaTab();    ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Stack"))     { StackTab();    ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Pool"))      { PoolTab();     ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("FreeList"))  { FreeListTab(); ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Benchmark")) { BenchTab();    ImGui::EndTabItem(); }
        ImGui::EndTabBar();
    }
}

void AllocatorScene::ArenaTab() {
    ImGui::TextWrapped("Bump a cursor; reclaim everything at once with Reset. No per-object free.");
    if (ImGui::Button("Alloc 8 KB")) m_arena.Allocate(8 * 1024, 16);
    ImGui::SameLine();
    if (ImGui::Button("Reset"))      m_arena.Reset();
    UsageBar(m_arena.Used(), m_arena.Capacity());
}

void AllocatorScene::StackTab() {
    ImGui::TextWrapped("Like the arena, but markers let you free in LIFO order: push a marker, "
                       "allocate scratch, pop to roll the cursor back.");
    if (ImGui::Button("Alloc 8 KB"))   m_stack.Allocate(8 * 1024, 16);
    ImGui::SameLine();
    if (ImGui::Button("Push marker"))  m_markers.push_back(m_stack.GetMarker());
    ImGui::SameLine();
    if (ImGui::Button("Pop marker") && !m_markers.empty()) {
        m_stack.FreeToMarker(m_markers.back());
        m_markers.pop_back();
    }
    UsageBar(m_stack.Used(), m_stack.Capacity());
    ImGui::Text("Markers on stack: %zu", m_markers.size());
}

void AllocatorScene::PoolTab() {
    ImGui::TextWrapped("Fixed-size blocks via an intrusive free list — O(1) alloc/free, no "
                       "fragmentation. Each square is a 64-byte slot.");
    if (ImGui::Button("Alloc block")) { if (void* p = m_pool.Allocate()) m_poolPtrs.push_back(p); }
    ImGui::SameLine();
    if (ImGui::Button("Alloc x32")) {
        for (int i = 0; i < 32; ++i) if (void* p = m_pool.Allocate()) m_poolPtrs.push_back(p);
    }
    ImGui::SameLine();
    if (ImGui::Button("Free random") && !m_poolPtrs.empty()) {
        std::uniform_int_distribution<size_t> d(0, m_poolPtrs.size() - 1);
        const size_t i = d(m_rng);
        m_pool.Free(m_poolPtrs[i]);
        m_poolPtrs.erase(m_poolPtrs.begin() + i);
    }
    ImGui::SameLine();
    if (ImGui::Button("Free all")) {
        for (void* p : m_poolPtrs) m_pool.Free(p);
        m_poolPtrs.clear();
    }
    ImGui::Text("Used: %zu / %zu blocks", m_pool.UsedCount(), m_pool.BlockCount());
    DrawPoolGrid();
}

void AllocatorScene::FreeListTab() {
    ImGui::TextWrapped("Variable-size general allocator: first-fit, split on alloc, coalesce "
                       "neighbours on free. Blue = an allocation, grey = free space.");
    auto allocRandom = [&] {
        std::uniform_int_distribution<int> d(1, 16);
        if (void* p = m_freelist.Allocate(d(m_rng) * 256)) m_flPtrs.push_back(p);
    };
    if (ImGui::Button("Alloc random")) allocRandom();
    ImGui::SameLine();
    if (ImGui::Button("Alloc x16")) for (int i = 0; i < 16; ++i) allocRandom();
    ImGui::SameLine();
    if (ImGui::Button("Free random") && !m_flPtrs.empty()) {
        std::uniform_int_distribution<size_t> d(0, m_flPtrs.size() - 1);
        const size_t i = d(m_rng);
        m_freelist.Free(m_flPtrs[i]);
        m_flPtrs.erase(m_flPtrs.begin() + i);
    }
    ImGui::SameLine();
    if (ImGui::Button("Free every other")) {
        for (size_t i = m_flPtrs.size(); i-- > 0; ) {
            if (i & 1) { m_freelist.Free(m_flPtrs[i]); m_flPtrs.erase(m_flPtrs.begin() + i); }
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Free all")) {
        for (void* p : m_flPtrs) m_freelist.Free(p);
        m_flPtrs.clear();
    }

    const std::size_t total   = m_freelist.Capacity();
    const std::size_t used    = m_freelist.UsedBytes();
    const std::size_t freeB   = m_freelist.FreeBytes();
    const std::size_t largest = m_freelist.LargestFreeBlock();
    const float fragPct = (freeB > 0)
        ? 100.0f * (1.0f - static_cast<float>(largest) / static_cast<float>(freeB)) : 0.0f;
    ImGui::Text("Used %.1f KB | Free %.1f KB | Largest run %.1f KB | %u allocs",
                used / 1024.0, freeB / 1024.0, largest / 1024.0, (unsigned)m_freelist.AllocationCount());
    ImGui::Text("Fragmentation: %.1f%%", fragPct);
    DrawFreeListMap();
}

void AllocatorScene::DrawPoolGrid() {
    std::vector<std::uint8_t> usage;
    m_pool.GetSlotUsage(usage);
    const int cols = 32;
    const int rows = static_cast<int>((usage.size() + cols - 1) / cols);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const float cell = 12.0f, pad = 2.0f;
    ImGui::Dummy(ImVec2(cols * cell, rows * cell));
    for (size_t i = 0; i < usage.size(); ++i) {
        const int cx = static_cast<int>(i % cols), cy = static_cast<int>(i / cols);
        const ImVec2 a(origin.x + cx * cell, origin.y + cy * cell);
        const ImVec2 b(a.x + cell - pad, a.y + cell - pad);
        dl->AddRectFilled(a, b, usage[i] ? IM_COL32(70, 150, 230, 255) : IM_COL32(45, 45, 52, 255));
    }
}

void AllocatorScene::DrawFreeListMap() {
    std::vector<Mem::FreeListAllocator::BlockInfo> blocks;
    m_freelist.GetBlocks(blocks);
    const std::size_t total = m_freelist.Capacity();
    if (total == 0) return;

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const float fullW = ImGui::GetContentRegionAvail().x;
    const float h = 44.0f;
    ImGui::Dummy(ImVec2(fullW, h));
    dl->AddRectFilled(origin, ImVec2(origin.x + fullW, origin.y + h), IM_COL32(20, 20, 24, 255));

    int usedIdx = 0;
    for (const auto& b : blocks) {
        float x0 = origin.x + fullW * static_cast<float>(static_cast<double>(b.offset) / total);
        float x1 = origin.x + fullW * static_cast<float>(static_cast<double>(b.offset + b.size) / total);
        if (x1 - x0 < 1.0f) x1 = x0 + 1.0f;
        ImU32 col;
        if (b.free) col = IM_COL32(45, 45, 52, 255);
        else {
            float hue = usedIdx * 0.13f; hue -= std::floor(hue);
            col = ImGui::ColorConvertFloat4ToU32(ImColor::HSV(hue, 0.55f, 0.9f));
            ++usedIdx;
        }
        dl->AddRectFilled(ImVec2(x0, origin.y), ImVec2(x1, origin.y + h), col);
        dl->AddRect(ImVec2(x0, origin.y), ImVec2(x1, origin.y + h), IM_COL32(0, 0, 0, 120));
    }
}

void AllocatorScene::BenchTab() {
    ImGui::Text("Correctness:");
    if (ImGui::BeginTable("alloc_correct", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Allocator");
        ImGui::TableSetupColumn("Result");
        ImGui::TableHeadersRow();
        for (const auto& c : m_results.correctness) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextUnformatted(c.name);
            ImGui::TableNextColumn();
            if (c.passed) ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.4f, 1), "PASS");
            else          ImGui::TextColored(ImVec4(0.95f, 0.4f, 0.4f, 1), "FAIL");
        }
        ImGui::EndTable();
    }

    ImGui::Separator();
    if (ImGui::Button("Run benchmark vs malloc")) {
        m_results  = Mem::Run(true);
        m_hasBench = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(Release build for representative numbers)");

    if (m_hasBench && !m_results.bench.empty()) {
        ImGui::Spacing();
        ImGui::Text("Timing — best of 16 (lower is better):");
        if (ImGui::BeginTable("alloc_bench", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Workload");
            ImGui::TableSetupColumn("malloc (ms)");
            ImGui::TableSetupColumn("custom (ms)");
            ImGui::TableSetupColumn("speedup");
            ImGui::TableHeadersRow();
            for (const auto& b : m_results.bench) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn(); ImGui::TextUnformatted(b.name);
                ImGui::TableNextColumn(); ImGui::Text("%.2f", b.mallocMs);
                ImGui::TableNextColumn(); ImGui::Text("%.2f", b.customMs);
                ImGui::TableNextColumn();
                if (b.customMs > 0.0) ImGui::Text("%.1fx", b.mallocMs / b.customMs);
                else                  ImGui::TextUnformatted("-");
            }
            ImGui::EndTable();
        }
    }
}
