#pragma once
// ---------------------------------------------------------------------------
// CpuProfiler — named CPU scope timing, the cheap sibling of GpuProfiler.
//
// Unlike GPU timing, CPU work finishes synchronously, so a scope's duration is
// known the instant it ends: no query heap, no deferred read-back. Regions may
// nest and carry a depth so the overlay can indent them the same way. The full
// frame's regions are collected into m_pending, then published to Results() at
// the next BeginFrame so the UI reads a stable, complete list.
//
// Use the ScopedCpuTimer RAII helper for exception-safe begin/end:
//     { SGE::ScopedCpuTimer t(profiler, "Update"); ...work... }
// ---------------------------------------------------------------------------
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace SGE {

class CpuProfiler {
public:
    using Clock = std::chrono::steady_clock;

    // Publish the frame just finished and start collecting a new one.
    void BeginFrame() {
        m_results.clear();
        m_results.reserve(m_pending.size());
        for (const Pending& p : m_pending)
            m_results.push_back({ p.name, p.ms, p.depth });
        m_pending.clear();
        m_depth = 0;
    }

    uint32_t Begin(const char* name) {
        const uint32_t id = static_cast<uint32_t>(m_pending.size());
        m_pending.push_back({ name, 0.0, m_depth++, Clock::now() });
        return id;
    }

    void End(uint32_t id) {
        const auto now = Clock::now();
        if (m_depth > 0) --m_depth;
        if (id < m_pending.size()) {
            const auto dt = now - m_pending[id].start;
            m_pending[id].ms =
                std::chrono::duration<double, std::milli>(dt).count();
        }
    }

    struct Scope {
        std::string name;
        double      ms    = 0.0;
        int         depth = 0;
    };
    const std::vector<Scope>& Results() const { return m_results; }

private:
    struct Pending {
        std::string      name;
        double           ms    = 0.0;
        int              depth = 0;
        Clock::time_point start;
    };
    std::vector<Pending> m_pending;   // regions being collected this frame
    std::vector<Scope>   m_results;   // last completed frame, for display
    int                  m_depth = 0;
};

// RAII begin/end so a scope is timed even if the body throws or early-returns.
class ScopedCpuTimer {
public:
    ScopedCpuTimer(CpuProfiler& p, const char* name)
        : m_p(p), m_id(p.Begin(name)) {}
    ~ScopedCpuTimer() { m_p.End(m_id); }
    ScopedCpuTimer(const ScopedCpuTimer&) = delete;
    ScopedCpuTimer& operator=(const ScopedCpuTimer&) = delete;
private:
    CpuProfiler& m_p;
    uint32_t     m_id;
};

} // namespace SGE
