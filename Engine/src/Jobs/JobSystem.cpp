#include "Jobs/JobSystem.h"
#include "Core/Logger.h"

#include <algorithm>

namespace SGE {

// ---------------- WorkStealingQueue ----------------

void WorkStealingQueue::Push(Job job)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_jobs.push_back(std::move(job));
}

bool WorkStealingQueue::Pop(Job& out)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_jobs.empty()) return false;
    out = std::move(m_jobs.back());
    m_jobs.pop_back();
    return true;
}

bool WorkStealingQueue::Steal(Job& out)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_jobs.empty()) return false;
    out = std::move(m_jobs.front());
    m_jobs.pop_front();
    return true;
}

bool WorkStealingQueue::Empty() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_jobs.empty();
}

// ---------------- JobSystem ----------------

void JobSystem::Initialize(uint32_t maxThreads)
{
    uint32_t hw = std::thread::hardware_concurrency();
    if (hw == 0) hw = 1;
    // Leave one core for the main thread; always have at least one worker.
    uint32_t want = (maxThreads == 0) ? (hw > 1 ? hw - 1 : 1) : maxThreads;

    m_threadCount = want;
    m_queueCount  = want;

    m_queues.clear();
    m_queues.reserve(m_queueCount);
    for (uint32_t i = 0; i < m_queueCount; ++i)
        m_queues.push_back(std::make_unique<WorkStealingQueue>());

    // +1 slot for the main thread (it runs jobs inside Wait()).
    m_executed = std::vector<std::atomic<uint64_t>>(m_threadCount + 1);
    m_steals   = std::vector<std::atomic<uint64_t>>(m_threadCount + 1);

    m_pending.store(0, std::memory_order_relaxed);
    m_pushCursor.store(0, std::memory_order_relaxed);
    m_alive.store(true, std::memory_order_release);

    m_threads.reserve(m_threadCount);
    for (uint32_t i = 0; i < m_threadCount; ++i)
        m_threads.emplace_back([this, i]() { WorkerLoop(i); });

    LogInfo("JobSystem started.");
}

void JobSystem::Shutdown()
{
    if (!m_alive.exchange(false)) return; // never initialized / already down

    // Wake every sleeping worker so it can observe m_alive == false and exit.
    {
        std::lock_guard<std::mutex> lock(m_wakeMutex);
    }
    m_wake.notify_all();

    for (auto& t : m_threads)
        if (t.joinable()) t.join();

    m_threads.clear();
    m_queues.clear();
    m_threadCount = 0;
    m_queueCount  = 0;

    LogInfo("JobSystem stopped.");
}

void JobSystem::Execute(Job job)
{
    // Count it as outstanding BEFORE it can possibly run, so Wait() never returns early.
    m_pending.fetch_add(1, std::memory_order_relaxed);

    Job wrapped = [this, j = std::move(job)]() mutable
    {
        j();
        m_pending.fetch_sub(1, std::memory_order_release);
    };

    if (m_queueCount == 0)
    {
        // No worker threads (single-core / not initialized): run inline.
        wrapped();
        return;
    }

    uint32_t q = m_pushCursor.fetch_add(1, std::memory_order_relaxed) % m_queueCount;
    m_queues[q]->Push(std::move(wrapped));

    // Acquiring m_wakeMutex here (after the push + pending increment) closes the
    // lost-wakeup gap against a worker that just evaluated its wait predicate.
    {
        std::lock_guard<std::mutex> lock(m_wakeMutex);
    }
    m_wake.notify_one();
}

void JobSystem::Dispatch(uint32_t jobCount, uint32_t groupSize,
                         const std::function<void(uint32_t)>& job)
{
    if (jobCount == 0) return;
    if (groupSize == 0) groupSize = 1;

    const uint32_t groupCount = (jobCount + groupSize - 1) / groupSize;
    for (uint32_t g = 0; g < groupCount; ++g)
    {
        Execute([job, g, groupSize, jobCount]()
        {
            const uint32_t begin = g * groupSize;
            const uint32_t end   = std::min(begin + groupSize, jobCount);
            for (uint32_t i = begin; i < end; ++i)
                job(i);
        });
    }
}

bool JobSystem::TryRunOneJob(int preferredQueue, uint32_t runner)
{
    Job job;

    // 1) Owner path: take from the front-runner's own queue (LIFO).
    if (preferredQueue >= 0 && m_queues[preferredQueue]->Pop(job))
    {
        job();
        m_executed[runner].fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    // 2) Steal path: scan the other queues (FIFO). Start just past the preferred
    //    one so workers don't all hammer queue 0.
    for (uint32_t i = 0; i < m_queueCount; ++i)
    {
        uint32_t q = (preferredQueue >= 0)
                       ? (uint32_t(preferredQueue) + 1 + i) % m_queueCount
                       : i;
        if (m_queues[q]->Steal(job))
        {
            job();
            m_executed[runner].fetch_add(1, std::memory_order_relaxed);
            m_steals[runner].fetch_add(1, std::memory_order_relaxed);
            return true;
        }
    }
    return false;
}

void JobSystem::WorkerLoop(uint32_t index)
{
    while (m_alive.load(std::memory_order_acquire))
    {
        if (!TryRunOneJob(int(index), index))
        {
            // Nothing to do — sleep until woken by new work or shutdown.
            std::unique_lock<std::mutex> lock(m_wakeMutex);
            m_wake.wait(lock, [this]()
            {
                return !m_alive.load(std::memory_order_acquire)
                    || m_pending.load(std::memory_order_acquire) > 0;
            });
        }
    }
}

void JobSystem::Wait()
{
    // The main thread helps: it steals and runs jobs (crediting the main slot)
    // until the outstanding count hits zero. Yielding when it finds nothing avoids
    // burning a core while workers finish the tail.
    const uint32_t mainSlot = m_threadCount; // last stats slot
    while (m_pending.load(std::memory_order_acquire) > 0)
    {
        if (!TryRunOneJob(-1, mainSlot))
            std::this_thread::yield();
    }
}

uint64_t JobSystem::Executed(uint32_t runner) const
{
    return runner < m_executed.size() ? m_executed[runner].load(std::memory_order_relaxed) : 0;
}

uint64_t JobSystem::Steals(uint32_t runner) const
{
    return runner < m_steals.size() ? m_steals[runner].load(std::memory_order_relaxed) : 0;
}

void JobSystem::ResetStats()
{
    for (auto& a : m_executed) a.store(0, std::memory_order_relaxed);
    for (auto& a : m_steals)   a.store(0, std::memory_order_relaxed);
}

} // namespace SGE
