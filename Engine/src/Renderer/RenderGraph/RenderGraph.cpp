#include "Renderer/RenderGraph/RenderGraph.h"
#include "Profiling/GpuProfiler.h"

namespace SGE {

D3D12_RESOURCE_STATES RenderGraph::ToD3D(RgState s)
{
    switch (s) {
        case RgState::RenderTarget:    return D3D12_RESOURCE_STATE_RENDER_TARGET;
        case RgState::DepthWrite:      return D3D12_RESOURCE_STATE_DEPTH_WRITE;
        case RgState::DepthRead:       return D3D12_RESOURCE_STATE_DEPTH_READ;
        case RgState::PixelShader:     return D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
        case RgState::NonPixelShader:  return D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
        case RgState::UnorderedAccess: return D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        case RgState::CopySrc:         return D3D12_RESOURCE_STATE_COPY_SOURCE;
        case RgState::CopyDst:         return D3D12_RESOURCE_STATE_COPY_DEST;
        case RgState::Common:          return D3D12_RESOURCE_STATE_COMMON;
    }
    return D3D12_RESOURCE_STATE_COMMON;
}

void RenderGraph::Begin(ID3D12GraphicsCommandList* cmd)
{
    m_cmd = cmd;
    m_resources.clear();
    m_passes.clear();
    // m_stateCache persists across frames on purpose (see header).
}

RgHandle RenderGraph::Import(const char* name, ID3D12Resource* res, RgState initial)
{
    // De-dup within a frame: importing the same resource twice returns the
    // handle already registered, so callers can Import defensively.
    for (uint32_t i = 0; i < m_resources.size(); ++i)
        if (m_resources[i].res == res)
            return { i };

    Resource r;
    r.name    = name;
    r.res     = res;
    r.initial = initial;
    m_resources.push_back(std::move(r));
    return { static_cast<uint32_t>(m_resources.size() - 1) };
}

void RenderGraph::AddPass(const char* name, std::vector<RgAccess> accesses,
                          ExecuteFn execute, bool sideEffect)
{
    Pass p;
    p.name       = name;
    p.access     = std::move(accesses);
    p.execute    = std::move(execute);
    p.sideEffect = sideEffect;
    m_passes.push_back(std::move(p));
}

void RenderGraph::Execute()
{
    // ---------------------------------------------------------------------
    // 1. Reference-count culling (Frostbite-style).
    //    resource.readRefs = number of passes that read it.
    //    pass.writeRefs    = number of resources it writes (its outputs).
    //    A resource with zero readers is "dead"; popping it decrements its
    //    producing pass's live-output count. When a pass has no live outputs
    //    left (and no side effect), it is culled and its own reads are released,
    //    which can cascade further culling upstream.
    //
    //    NOTE: producer = the LAST pass to write a resource. That is exact for
    //    the linear pipelines here (no resource is written by two passes). True
    //    multi-writer / versioned resources are a Phase-2 concern.
    // ---------------------------------------------------------------------
    for (auto& r : m_resources) { r.readRefs = 0; r.producer = -1; }

    for (uint32_t pi = 0; pi < m_passes.size(); ++pi) {
        Pass& p = m_passes[pi];
        p.culled    = false;
        p.writeRefs = 0;
        for (const RgAccess& a : p.access) {
            if (!a.resource.IsValid()) continue;
            Resource& r = m_resources[a.resource.id];
            if (a.write) { r.producer = static_cast<int>(pi); ++p.writeRefs; }
            else         { ++r.readRefs; }
        }
    }

    // Seed the worklist with resources nobody reads.
    std::vector<uint32_t> dead;
    for (uint32_t ri = 0; ri < m_resources.size(); ++ri)
        if (m_resources[ri].readRefs == 0)
            dead.push_back(ri);

    while (!dead.empty()) {
        const uint32_t ri = dead.back();
        dead.pop_back();
        const int prod = m_resources[ri].producer;
        if (prod < 0) continue;               // imported input with no producer
        Pass& p = m_passes[prod];
        if (p.sideEffect || p.culled) continue;
        if (--p.writeRefs != 0) continue;     // still has other live outputs
        p.culled = true;
        // Releasing this pass frees its inputs; any that hit zero readers cascade.
        for (const RgAccess& a : p.access) {
            if (a.write || !a.resource.IsValid()) continue;
            Resource& r = m_resources[a.resource.id];
            if (r.readRefs > 0 && --r.readRefs == 0)
                dead.push_back(a.resource.id);
        }
    }

    // ---------------------------------------------------------------------
    // 2. Execute survivors in order, deriving barriers by diffing state.
    // ---------------------------------------------------------------------
    std::vector<D3D12_RESOURCE_STATES> current(m_resources.size());
    for (uint32_t ri = 0; ri < m_resources.size(); ++ri) {
        auto it = m_stateCache.find(m_resources[ri].res);
        current[ri] = (it != m_stateCache.end()) ? it->second
                                                  : ToD3D(m_resources[ri].initial);
    }

    m_schedule.clear();
    m_schedule.reserve(m_passes.size());
    m_totalBarriers = 0;

    std::vector<D3D12_RESOURCE_BARRIER> barriers;
    for (Pass& p : m_passes) {
        PassInfo info;
        info.name       = p.name;
        info.sideEffect = p.sideEffect;
        info.culled     = p.culled;
        info.accesses   = static_cast<uint32_t>(p.access.size());

        if (p.culled) { m_schedule.push_back(std::move(info)); continue; }

        // Collect the transitions this pass needs. Multiple accesses to the same
        // resource (rare) resolve left-to-right against the running state.
        barriers.clear();
        for (const RgAccess& a : p.access) {
            if (!a.resource.IsValid()) continue;
            const D3D12_RESOURCE_STATES need = ToD3D(a.state);
            D3D12_RESOURCE_STATES& cur = current[a.resource.id];
            if (cur == need) continue;

            D3D12_RESOURCE_BARRIER b = {};
            b.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            b.Flags                  = D3D12_RESOURCE_BARRIER_FLAG_NONE;
            b.Transition.pResource   = m_resources[a.resource.id].res;
            b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            b.Transition.StateBefore = cur;
            b.Transition.StateAfter  = need;
            barriers.push_back(b);
            cur = need;
        }

        if (!barriers.empty())
            m_cmd->ResourceBarrier(static_cast<UINT>(barriers.size()), barriers.data());

        info.barriers = static_cast<uint32_t>(barriers.size());
        m_totalBarriers += info.barriers;
        // GPU time is ~FrameCount frames stale (async read-back), so report the
        // last-known value for this pass name from the profiler's results.
        if (m_profiler)
            info.gpuMs = m_profiler->LastRegionMs(p.name.c_str());
        m_schedule.push_back(std::move(info));

        // Time the pass's recorded work (its draws/dispatches) if a profiler is
        // attached; the preceding barriers are negligible and excluded.
        const uint32_t region = m_profiler
            ? m_profiler->BeginRegion(m_cmd, p.name.c_str())
            : GpuProfiler::kInvalid;
        if (p.execute)
            p.execute(m_cmd);
        if (m_profiler)
            m_profiler->EndRegion(m_cmd, region);
    }

    // 3. Remember where every resource ended up, for next frame's imports.
    for (uint32_t ri = 0; ri < m_resources.size(); ++ri)
        m_stateCache[m_resources[ri].res] = current[ri];
}

} // namespace SGE
