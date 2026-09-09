#pragma once
// ---------------------------------------------------------------------------
// RenderGraph — a per-frame, declarative pass scheduler.
//
// The problem it solves: today every scene manages DX12 resource-state barriers
// by hand (GBuffer::TransitionToRenderTargets, RenderTexture::TransitionTo, ...),
// calling them in exactly the right order. Forget one, or reorder two passes,
// and you get a silent GPU hazard the debug layer only *sometimes* catches.
//
// With a graph, a pass instead DECLARES which resources it reads/writes and the
// state it needs each in. The graph then:
//   * derives every barrier by diffing each resource's current vs required state
//     (batched into one ResourceBarrier per pass) — barriers become a computed
//     fact, not something the author remembers to write;
//   * culls passes whose outputs nobody consumes (reference counting), so dead
//     work — and its barriers — simply never record.
//
// It is intentionally a *scheduling* layer, not a resource *owner*: it references
// resources the engine already owns (the back buffer, the depth buffer, a
// GBuffer's textures, a RenderTexture) and leaves descriptor/RTV binding to the
// pass body using the existing helpers. That keeps this a drop-in over the
// current code rather than a rewrite.
//
// PHASE 2 (not yet implemented): graph-OWNED transient resources — passes declare
// a texture by descriptor, the graph allocates it from a pool and ALIASES memory
// between resources whose lifetimes don't overlap (two targets sharing the same
// heap bytes). That is the memory story; it only pays off once this scheduling
// layer exists and is proven, which is why it is deferred. The Import-based API
// below is the seam it will slot into (add an AllocateTransient() returning the
// same RgHandle; culling + barriers already work unchanged).
// ---------------------------------------------------------------------------
#include "Renderer/DX12/DX12Common.h"

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace SGE {

class GpuProfiler;

// How a pass uses a resource. Each maps to one D3D12_RESOURCE_STATES (see ToD3D).
enum class RgState {
    RenderTarget,    // written via OMSetRenderTargets
    DepthWrite,      // depth/stencil target, writes enabled
    DepthRead,       // depth bound read-only (e.g. depth test, no write)
    PixelShader,     // sampled in a pixel shader (SRV)
    NonPixelShader,  // sampled in a non-pixel stage (e.g. compute SRV)
    UnorderedAccess, // UAV (compute read/write) — for future compute passes
    CopySrc,
    CopyDst,
    Common,
};

// Opaque handle into a RenderGraph's per-frame resource table.
struct RgHandle {
    uint32_t id = UINT32_MAX;
    bool IsValid() const { return id != UINT32_MAX; }
};

// One declared resource access: which resource, the state it must be in for the
// pass, and whether the pass writes it (writes make the pass a producer, which
// is what keeps it alive during culling).
struct RgAccess {
    RgHandle resource;
    RgState  state;
    bool     write;
};

// Readable helpers for the initializer-list pass declaration:
//   AddPass("Lighting", { Read(gbuffer, PixelShader), Write(back, RenderTarget) }, ...)
inline RgAccess Read (RgHandle h, RgState s) { return { h, s, false }; }
inline RgAccess Write(RgHandle h, RgState s) { return { h, s, true  }; }

class RenderGraph {
public:
    using ExecuteFn = std::function<void(ID3D12GraphicsCommandList*)>;

    // Start recording this frame's graph into `cmd`. Clears the per-frame pass
    // and resource lists but KEEPS the cross-frame state cache, so an imported
    // resource remembers the state the previous frame left it in.
    void Begin(ID3D12GraphicsCommandList* cmd);

    // Register an engine-owned resource for the graph to transition. `initial`
    // is used only the first time this exact resource pointer is seen; after
    // that the graph's own cache is the source of truth. Re-importing the same
    // pointer within a frame returns the existing handle.
    RgHandle Import(const char* name, ID3D12Resource* res, RgState initial);

    // Declare a pass: its resource accesses and the callback that records its
    // draws/dispatches. Set sideEffect=true to pin a pass so culling never
    // removes it — used for the pass that writes the swap-chain back buffer (its
    // result leaves the graph, so the graph can't see that it's consumed).
    void AddPass(const char* name, std::vector<RgAccess> accesses,
                 ExecuteFn execute, bool sideEffect = false);

    // Optional GPU profiler: when set, each surviving pass is wrapped in a
    // timing region named after the pass, so per-pass GPU cost lands in the
    // profiler results (and this frame's PassInfo::gpuMs) for free.
    void SetProfiler(GpuProfiler* p) { m_profiler = p; }

    // Reference-count cull unused passes, then run the survivors in declared
    // order — emitting one batched ResourceBarrier per pass for the states that
    // actually changed — and write final states back into the cross-frame cache.
    void Execute();

    // Drop the cross-frame state cache. Call when imported resources are
    // recreated (e.g. a GBuffer rebuilt on window resize), since a freed
    // ID3D12Resource address could otherwise be reused and matched to a stale
    // cached state.
    void ResetStateCache() { m_stateCache.clear(); }

    // ---- Introspection, for the RenderGraphScene inspector panel ----
    struct PassInfo {
        std::string name;
        bool        sideEffect = false;
        bool        culled     = false;
        uint32_t    accesses   = 0;  // declared reads+writes
        uint32_t    barriers   = 0;  // transitions the graph emitted before it
        double      gpuMs      = -1.0; // last-known GPU time (needs a profiler)
    };
    const std::vector<PassInfo>& LastSchedule() const { return m_schedule; }
    uint32_t LastBarrierCount() const { return m_totalBarriers; }

private:
    struct Resource {
        std::string     name;
        ID3D12Resource* res      = nullptr;
        RgState         initial  = RgState::Common;
        int             producer = -1;  // index of the last pass that writes it
        uint32_t        readRefs = 0;   // # passes reading it (culling counter)
    };
    struct Pass {
        std::string           name;
        std::vector<RgAccess> access;
        ExecuteFn             execute;
        bool                  sideEffect = false;
        bool                  culled     = false;
        uint32_t              writeRefs  = 0;  // # live outputs (culling counter)
    };

    static D3D12_RESOURCE_STATES ToD3D(RgState s);

    ID3D12GraphicsCommandList* m_cmd      = nullptr;
    GpuProfiler*               m_profiler = nullptr;
    std::vector<Resource>      m_resources;
    std::vector<Pass>          m_passes;

    // Cross-frame: the state each imported resource was last left in.
    std::unordered_map<ID3D12Resource*, D3D12_RESOURCE_STATES> m_stateCache;

    // Introspection snapshot of the most recent Execute().
    std::vector<PassInfo> m_schedule;
    uint32_t              m_totalBarriers = 0;
};

} // namespace SGE
