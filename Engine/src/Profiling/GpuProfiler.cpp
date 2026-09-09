#include "Profiling/GpuProfiler.h"
#include "Core/Logger.h"

namespace SGE {

bool GpuProfiler::Initialize(ID3D12Device* device, ID3D12CommandQueue* queue)
{
    try {
        // Ticks-per-second for this queue's timestamp clock (fixed on DX12,
        // unlike D3D11's per-frame disjoint query).
        SGE_THROW_IF_FAILED(queue->GetTimestampFrequency(&m_freq));

        D3D12_QUERY_HEAP_DESC heapDesc = {};
        heapDesc.Type  = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
        heapDesc.Count = kMaxTimestamps;
        SGE_THROW_IF_FAILED(device->CreateQueryHeap(&heapDesc, IID_PPV_ARGS(&m_heap)));

        // READBACK buffer, one slot per frame in flight. Created in COPY_DEST
        // (the required and permanent state for a readback resource), so
        // ResolveQueryData can target it every frame with no barriers.
        D3D12_HEAP_PROPERTIES rb = {};
        rb.Type = D3D12_HEAP_TYPE_READBACK;

        D3D12_RESOURCE_DESC desc = {};
        desc.Dimension        = D3D12_RESOURCE_DIMENSION_BUFFER;
        desc.Width            = uint64_t(FrameCount) * kMaxTimestamps * sizeof(uint64_t);
        desc.Height           = 1;
        desc.DepthOrArraySize = 1;
        desc.MipLevels        = 1;
        desc.Format           = DXGI_FORMAT_UNKNOWN;
        desc.SampleDesc.Count = 1;
        desc.Layout           = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

        SGE_THROW_IF_FAILED(device->CreateCommittedResource(
            &rb, D3D12_HEAP_FLAG_NONE, &desc,
            D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&m_readback)));

        return true;
    }
    catch (const std::exception& e) {
        LogError(e.what());
        return false;
    }
}

void GpuProfiler::Shutdown()
{
    m_readback.Reset();
    m_heap.Reset();
    for (auto& f : m_frameRegions) f.clear();
    m_results.clear();
}

void GpuProfiler::BeginFrame(uint32_t frameIndex)
{
    m_frameIndex = frameIndex;
    m_cursor     = 0;
    m_depth      = 0;

    // Read back the results the GPU produced the last time this slot was used
    // (FrameCount frames ago). Renderer::BeginFrame already waited on this
    // slot's fence, so the timestamps are complete.
    std::vector<Pending>& layout = m_frameRegions[frameIndex];
    m_results.clear();
    if (!layout.empty() && m_readback && m_freq != 0) {
        const uint64_t slotBase   = uint64_t(frameIndex) * kMaxTimestamps;
        const uint64_t byteOffset = slotBase * sizeof(uint64_t);
        const uint64_t byteEnd    = byteOffset + uint64_t(kMaxTimestamps) * sizeof(uint64_t);

        D3D12_RANGE readRange{ SIZE_T(byteOffset), SIZE_T(byteEnd) };
        void* mapped = nullptr;
        if (SUCCEEDED(m_readback->Map(0, &readRange, &mapped))) {
            const uint64_t* ts = reinterpret_cast<const uint64_t*>(mapped);
            m_results.reserve(layout.size());
            for (const Pending& p : layout) {
                double ms = 0.0;
                if (p.end != kInvalid) {
                    const uint64_t t0 = ts[slotBase + p.begin];
                    const uint64_t t1 = ts[slotBase + p.end];
                    if (t1 > t0)
                        ms = double(t1 - t0) * 1000.0 / double(m_freq);
                }
                m_results.push_back({ p.name, ms, p.depth });
            }
            D3D12_RANGE noWrite{ 0, 0 }; // CPU wrote nothing back
            m_readback->Unmap(0, &noWrite);
        }
    }

    layout.clear(); // ready to record this frame's regions afresh
}

uint32_t GpuProfiler::BeginRegion(ID3D12GraphicsCommandList* cmd, const char* name)
{
    if (!m_heap || m_cursor >= kMaxTimestamps)
        return kInvalid;

    const uint32_t beginIdx = m_cursor++;
    cmd->EndQuery(m_heap.Get(), D3D12_QUERY_TYPE_TIMESTAMP, beginIdx);

    std::vector<Pending>& layout = m_frameRegions[m_frameIndex];
    const uint32_t id = static_cast<uint32_t>(layout.size());
    layout.push_back({ name, beginIdx, kInvalid, m_depth++ });
    return id;
}

void GpuProfiler::EndRegion(ID3D12GraphicsCommandList* cmd, uint32_t id)
{
    if (id == kInvalid || !m_heap)
        return;
    if (m_depth > 0) --m_depth;
    if (m_cursor >= kMaxTimestamps)
        return;

    const uint32_t endIdx = m_cursor++;
    cmd->EndQuery(m_heap.Get(), D3D12_QUERY_TYPE_TIMESTAMP, endIdx);

    std::vector<Pending>& layout = m_frameRegions[m_frameIndex];
    if (id < layout.size())
        layout[id].end = endIdx;
}

void GpuProfiler::Resolve(ID3D12GraphicsCommandList* cmd)
{
    if (!m_heap || !m_readback || m_cursor == 0)
        return;
    const uint64_t byteOffset =
        uint64_t(m_frameIndex) * kMaxTimestamps * sizeof(uint64_t);
    cmd->ResolveQueryData(m_heap.Get(), D3D12_QUERY_TYPE_TIMESTAMP,
                          0, m_cursor, m_readback.Get(), byteOffset);
}

double GpuProfiler::FrameGpuMs() const
{
    double total = 0.0;
    for (const Region& r : m_results)
        if (r.depth == 0) total += r.ms;
    return total;
}

double GpuProfiler::LastRegionMs(const char* name) const
{
    for (const Region& r : m_results)
        if (r.name == name) return r.ms;
    return -1.0;
}

} // namespace SGE
