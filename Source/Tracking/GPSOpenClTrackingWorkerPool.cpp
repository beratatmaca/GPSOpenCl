#include "Tracking/GPSOpenClTrackingWorkerPool.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

using namespace GPSOpenCl;

TrackingWorkerPool::TrackingWorkerPool(WorkFunction work, int maxWorkers) : m_work(std::move(work))
{
    const unsigned int hw = std::thread::hardware_concurrency();
    m_numWorkers = std::max(1, std::min(static_cast<int>(hw > 0 ? hw : 4), maxWorkers));
    m_workerDurationMs.assign(static_cast<size_t>(m_numWorkers), 0.0);

    if (m_numWorkers <= 1)
    {
        return;
    }

    m_workers.reserve(static_cast<size_t>(m_numWorkers));
    for (int i = 0; i < m_numWorkers; i++)
    {
        m_workers.emplace_back([this, i] { workerLoop(i); });
    }
}

TrackingWorkerPool::~TrackingWorkerPool()
{
    if (m_workers.empty())
    {
        return;
    }

    {
        const std::lock_guard<std::mutex> lock(m_mutex);
        m_shutdown = true;
    }
    m_startCv.notify_all();

    for (auto &worker : m_workers)
    {
        if (worker.joinable())
        {
            worker.join();
        }
    }
    m_workers.clear();
}

void TrackingWorkerPool::run(int itemCount)
{
    if (itemCount <= 0)
    {
        return;
    }

    std::fill(m_workerDurationMs.begin(), m_workerDurationMs.end(), 0.0);

    const int helpers = m_workers.empty() ? 0 : std::min(m_numWorkers, itemCount - 1);
    if (helpers <= 0)
    {
        auto workStart = std::chrono::high_resolution_clock::now();
        m_itemCount = itemCount;
        m_cursor.store(0, std::memory_order_relaxed);
        drainCursor();
        auto workEnd = std::chrono::high_resolution_clock::now();
        m_callerDurationMs = std::chrono::duration<double, std::milli>(workEnd - workStart).count();
        return;
    }

    {
        const std::lock_guard<std::mutex> lock(m_mutex);
        m_itemCount = itemCount;
        m_cursor.store(0, std::memory_order_relaxed);
        m_pendingWorkers = helpers;
        m_helperTarget = helpers;
        m_claimedWorkers = 0;
        m_generation++;
    }
    m_startCv.notify_all();

    auto workStart = std::chrono::high_resolution_clock::now();
    drainCursor();
    auto workEnd = std::chrono::high_resolution_clock::now();
    m_callerDurationMs = std::chrono::duration<double, std::milli>(workEnd - workStart).count();

    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_doneCv.wait(lock, [&] { return m_pendingWorkers == 0; });
    }
}

double TrackingWorkerPool::maxWorkerDurationMs() const
{
    double worst = m_callerDurationMs;
    for (const double duration : m_workerDurationMs)
    {
        worst = std::max(worst, duration);
    }
    return worst;
}

void TrackingWorkerPool::workerLoop(int workerIndex)
{
    uint64_t lastSeenGeneration = 0;

    while (true)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_startCv.wait(lock, [&] { return m_shutdown || m_generation != lastSeenGeneration; });
        if (m_shutdown)
        {
            return;
        }
        lastSeenGeneration = m_generation;
        if (m_claimedWorkers >= m_helperTarget)
        {
            continue;
        }
        m_claimedWorkers++;
        lock.unlock();

        auto workStart = std::chrono::high_resolution_clock::now();
        drainCursor();
        auto workEnd = std::chrono::high_resolution_clock::now();
        m_workerDurationMs[static_cast<size_t>(workerIndex)] = std::chrono::duration<double, std::milli>(workEnd - workStart).count();

        lock.lock();
        m_pendingWorkers--;
        if (m_pendingWorkers == 0)
        {
            lock.unlock();
            m_doneCv.notify_one();
        }
    }
}

void TrackingWorkerPool::drainCursor()
{
    while (true)
    {
        const int item = m_cursor.fetch_add(1, std::memory_order_relaxed);
        if (item >= m_itemCount)
        {
            return;
        }
        m_work(item);
    }
}
