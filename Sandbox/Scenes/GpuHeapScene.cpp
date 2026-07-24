#include "Scenes/GpuHeapScene.h"

#include "imgui.h"
#include <cmath>

using namespace SGE;

static constexpr uint64_t kHeapSize = 16ull * 1024 * 1024; // 16 MB = 256 × 64 KB blocks
static constexpr uint64_t kBlock    = 64ull * 1024;        // 64 KB allocation unit

const char* GpuHeapScene::Description() const {
    return "One 16 MB ID3D12Heap, sub-allocated into PLACED buffer resources by a "
           "first-fit free-list allocator (split on alloc, coalesce on free). Use "
           "the buttons to allocate/free and watch the heap map and fragmentation "
           "change. Blue = a placed resource; grey = free space.";
}

void GpuHeapScene::OnLoad(const DemoContext& ctx) {
    m_device = ctx.device;
    m_heap.Init(ctx.device, kHeapSize, D3D12_HEAP_TYPE_DEFAULT);
    m_loaded = true;

    // Seed a few allocations so the heap map isn't empty on first view.
    for (int i = 0; i < 4; ++i)
        AllocRandom();
}

void GpuHeapScene::OnUnload() {
    FreeAll();
    m_heap.Shutdown();
    m_loaded = false;
}

void GpuHeapScene::DoAlloc(uint64_t bytes) {
    // These resources are never bound to a draw, so the GPU never touches them —
    // allocate/free freely without fencing. (Real bound resources would need a
    // WaitForGPU before their region could be reused.)
    GpuAllocation a = m_heap.AllocateBuffer(m_device, bytes);
    if (a.IsValid())
        m_allocs.push_back(std::move(a));
}

void GpuHeapScene::AllocRandom() {
    // 1..16 blocks => 64 KB .. 1 MB
    std::uniform_int_distribution<int> dist(1, 16);
    DoAlloc(static_cast<uint64_t>(dist(m_rng)) * kBlock);
}

void GpuHeapScene::FreeRandom() {
    if (m_allocs.empty())
        return;
    std::uniform_int_distribution<size_t> dist(0, m_allocs.size() - 1);
    const size_t i = dist(m_rng);
    m_heap.Free(m_allocs[i]);
    m_allocs.erase(m_allocs.begin() + i);
}

void GpuHeapScene::FreeAll() {
    for (auto& a : m_allocs)
        m_heap.Free(a);
    m_allocs.clear();
}

void GpuHeapScene::OnImGui() {
    if (!m_loaded)
        return;

    const uint64_t total   = m_heap.TotalSize();
    const uint64_t used    = m_heap.UsedBytes();
    const uint64_t freeB   = m_heap.FreeBytes();
    const uint64_t largest = m_heap.LargestFreeBlock();
    // Fragmentation: how much of the free space is NOT in the single largest
    // run. 0% = all free space contiguous; high = the heap is checker-boarded.
    const float fragPct = (freeB > 0)
        ? 100.0f * (1.0f - static_cast<float>(largest) / static_cast<float>(freeB))
        : 0.0f;

    const double MB = 1024.0 * 1024.0;
    ImGui::Text("Heap size:  %.0f MB  (%llu x 64 KB blocks)",
                total / MB, static_cast<unsigned long long>(total / kBlock));
    ImGui::Text("Used:       %.2f MB  (%u allocations)", used / MB, m_heap.AllocationCount());
    ImGui::Text("Free:       %.2f MB  | largest run %.2f MB", freeB / MB, largest / MB);
    ImGui::Text("Fragmentation: %.1f%%", fragPct);
    ImGui::ProgressBar(total ? static_cast<float>(static_cast<double>(used) / static_cast<double>(total)) : 0.0f,
                       ImVec2(-1.0f, 0.0f));

    ImGui::Separator();
    if (ImGui::Button("Alloc random")) AllocRandom();
    ImGui::SameLine();
    if (ImGui::Button("Alloc 1 MB"))   DoAlloc(1024 * 1024);
    ImGui::SameLine();
    if (ImGui::Button("Free random"))  FreeRandom();
    ImGui::SameLine();
    if (ImGui::Button("Free all"))     FreeAll();

    // Stress helper: many small allocs interleaved create visible fragmentation.
    if (ImGui::Button("Alloc x16")) {
        for (int i = 0; i < 16; ++i) AllocRandom();
    }
    ImGui::SameLine();
    if (ImGui::Button("Free every other")) {
        // Free odd-indexed allocations to deliberately fragment the heap.
        for (size_t i = m_allocs.size(); i-- > 0; ) {
            if (i & 1) {
                m_heap.Free(m_allocs[i]);
                m_allocs.erase(m_allocs.begin() + i);
            }
        }
    }

    ImGui::Separator();
    DrawHeapBar();
}

void GpuHeapScene::DrawHeapBar() {
    const auto& blocks = m_heap.Blocks();
    const uint64_t total = m_heap.TotalSize();
    if (total == 0)
        return;

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const float  fullW  = ImGui::GetContentRegionAvail().x;
    const float  height = 48.0f;
    ImGui::Dummy(ImVec2(fullW, height)); // reserve layout space for the custom draw

    // Backing rectangle.
    dl->AddRectFilled(origin, ImVec2(origin.x + fullW, origin.y + height), IM_COL32(20, 20, 24, 255));

    int usedIdx = 0;
    for (const auto& b : blocks) {
        float x0 = origin.x + fullW * static_cast<float>(static_cast<double>(b.Offset) / static_cast<double>(total));
        float x1 = origin.x + fullW * static_cast<float>(static_cast<double>(b.Offset + b.Size) / static_cast<double>(total));
        if (x1 - x0 < 1.0f) x1 = x0 + 1.0f; // keep single-block allocations visible

        ImU32 col;
        if (b.Free) {
            col = IM_COL32(45, 45, 52, 255);
        } else {
            // Distinct hue per placed resource so adjacent allocations read apart.
            float hue = usedIdx * 0.13f;
            hue -= std::floor(hue);
            col = ImGui::ColorConvertFloat4ToU32(ImColor::HSV(hue, 0.55f, 0.90f));
            ++usedIdx;
        }
        dl->AddRectFilled(ImVec2(x0, origin.y), ImVec2(x1, origin.y + height), col);
        dl->AddRect(ImVec2(x0, origin.y), ImVec2(x1, origin.y + height), IM_COL32(0, 0, 0, 120));
    }
}
