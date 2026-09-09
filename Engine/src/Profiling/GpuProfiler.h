#pragma once
// ---------------------------------------------------------------------------
// GpuProfiler — per-region GPU timing via DX12 timestamp queries.
//
// The GPU runs asynchronously, so you can't ask "how long did the shadow pass
// take" with a CPU timer — the CPU has moved on frames before the GPU gets
// there. Instead you write a TIMESTAMP query into the command stream at the
// start and end of a region; the GPU records its clock as it reaches each, and
// later you read the two values back and difference them.
//
// The catch is the read-back: the results aren't ready until the GPU actually
// finished the frame. This profiler resolves each frame's timestamps into a
// per-frame-slot region of a READBACK buffer, and reads a slot only FrameCount
// frames later — by which point Renderer::BeginFrame has already waited on that
// slot's fence, so the data is guaranteed complete and no stall is introduced.
// Reported timings are therefore ~FrameCount frames stale, which is invisible
// for a live profiler HUD.
//
// Regions may nest (a top-level "Scene" region containing the render graph's
// per-pass regions); each result carries a depth so the overlay can indent it.
// ---------------------------------------------------------------------------
#include "Renderer/DX12/DX12Common.h"

#include <cstdint>
#include <string>
#include <vector>

namespace SGE {

class GpuProfiler {
public:
    bool Initialize(ID3D12Device* device, ID3D12CommandQueue* queue);
    void Shutdown();

    // Start of frame: read back the results the GPU wrote for this slot
    // (FrameCount frames ago) and reset this frame's recording cursor.
    void BeginFrame(uint32_t frameIndex);

    // Open a timing region; returns an id to pass to EndRegion. No-op-safe if
    // the per-frame region budget is exceeded (returns kInvalid).
    uint32_t BeginRegion(ID3D12GraphicsCommandList* cmd, const char* name);
    void     EndRegion(ID3D12GraphicsCommandList* cmd, uint32_t id);

    // End of frame: copy this frame's timestamps from the query heap into the
    // readback buffer. Record after all regions, before the list is closed.
    void Resolve(ID3D12GraphicsCommandList* cmd);

    struct Region {
        std::string name;
        double      ms    = 0.0;
        int         depth = 0;
    };
    const std::vector<Region>& Results() const { return m_results; }
    double FrameGpuMs() const;                       // sum of depth-0 regions
    double LastRegionMs(const char* name) const;     // most recent match, or -1

    static constexpr uint32_t kInvalid = UINT32_MAX;

private:
    static constexpr uint32_t kMaxRegions    = 128;
    static constexpr uint32_t kMaxTimestamps = kMaxRegions * 2;

    // A region recorded this frame, resolved to ms when its slot is read back.
    struct Pending {
        std::string name;
        uint32_t    begin = 0;          // timestamp index for the start
        uint32_t    end   = kInvalid;   // timestamp index for the end
        int         depth = 0;
    };

    ComPtr<ID3D12QueryHeap> m_heap;
    ComPtr<ID3D12Resource>  m_readback;  // FrameCount * kMaxTimestamps uint64s
    uint64_t m_freq       = 0;           // timestamp ticks per second
    uint32_t m_frameIndex = 0;
    uint32_t m_cursor     = 0;           // timestamps used so far this frame
    int      m_depth      = 0;

    // Region layout per frame slot, kept until that slot is read back later.
    std::vector<Pending> m_frameRegions[FrameCount];
    std::vector<Region>  m_results;      // last slot read back, for display
};

} // namespace SGE
