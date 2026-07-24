#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace SGE {

using Job = std::function<void()>;

// ---------------------------------------------------------------------------
// WorkStealingQueue — one deque of jobs per worker.
//
// The concept: instead of a single global job queue (which every thread contends
// on with one lock), each worker owns its own queue. The owner pushes/pops at the
// BACK (LIFO — the most recently spawned job is usually the hottest in cache, and
// owner access never collides with thieves at the other end). Idle workers with
// nothing of their own STEAL from the FRONT of a peer's queue (FIFO — the oldest
// job, least likely to be cache-hot for its owner, and taken from the opposite end
// so it rarely contends with the owner).
//
// This is guarded by a plain mutex: simple and obviously correct, which is the
// point for a showcase. The performance stretch is a lock-free Chase-Lev deque.
// ---------------------------------------------------------------------------
class WorkStealingQueue
{
public:
    void Push(Job job);       // owner: append at back
    bool Pop(Job& out);       // owner: take from back (LIFO)
    bool Steal(Job& out);     // thief: take from front (FIFO)
    bool Empty() const;

private:
    mutable std::mutex m_mutex;
    std::deque<Job>    m_jobs;
};

// ---------------------------------------------------------------------------
// JobSystem — a work-stealing thread pool.
//
// Initialize() spins up (hardware threads - 1) workers, each with its own queue,
// leaving one core for the main thread. Execute()/Dispatch() hand out jobs
// round-robin across the worker queues; a worker that drains its own queue steals
// from its peers. Wait() blocks the CALLING (main) thread until every outstanding
// job has finished — and crucially it RUNS jobs while it waits, so the main thread
// pitches in instead of spinning idle, and there is never a deadlock where the main
// thread sleeps on work that only it could run.
//
// This is the engine's parallelism substrate: the threaded command-list demo uses
// it to record draws across cores, and future particles/physics fan out the same way.
// ---------------------------------------------------------------------------
class JobSystem
{
public:
    // maxThreads == 0 -> auto (hardware_concurrency - 1, at least 1).
    void Initialize(uint32_t maxThreads = 0);
    void Shutdown();

    // Schedule a single job.
    void Execute(Job job);

    // Parallel-for: split [0, jobCount) into groups of groupSize and schedule each
    // group as one job. The callback receives the flat job index.
    void Dispatch(uint32_t jobCount, uint32_t groupSize,
                  const std::function<void(uint32_t)>& job);

    // Block (and help execute) until all scheduled jobs have completed.
    void Wait();

    bool     IsBusy()      const { return m_pending.load(std::memory_order_acquire) > 0; }
    uint32_t ThreadCount() const { return m_threadCount; } // worker threads (excludes main)

    // --- stats for the ImGui visualizer ---
    // Runner slots: 0..ThreadCount()-1 are workers, ThreadCount() is the main thread.
    uint64_t Executed(uint32_t runner) const;
    uint64_t Steals(uint32_t runner)   const;
    void     ResetStats();

private:
    void WorkerLoop(uint32_t index);
    // Run one job: try the preferred queue first (owner), then steal from peers.
    // runner is the stats slot to credit. Returns false if no work was found.
    bool TryRunOneJob(int preferredQueue, uint32_t runner);

    std::vector<std::unique_ptr<WorkStealingQueue>> m_queues;   // one per worker
    std::vector<std::thread>                        m_threads;

    std::atomic<bool>     m_alive{false};
    std::atomic<uint32_t> m_pending{0};      // scheduled-but-unfinished job count
    std::atomic<uint32_t> m_pushCursor{0};   // round-robin distribution cursor
    uint32_t              m_threadCount = 0;
    uint32_t              m_queueCount  = 0;

    std::mutex              m_wakeMutex;
    std::condition_variable m_wake;

    // Sized ThreadCount()+1: last slot is the main thread. atomic so worker + main
    // updates don't race.
    std::vector<std::atomic<uint64_t>> m_executed;
    std::vector<std::atomic<uint64_t>> m_steals;
};

} // namespace SGE
