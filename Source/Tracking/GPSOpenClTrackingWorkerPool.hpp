#ifndef INCLUDED_GPSOPENCL_TRACKINGWORKERPOOL_HPP
#define INCLUDED_GPSOPENCL_TRACKINGWORKERPOOL_HPP

/** @file GPSOpenClTrackingWorkerPool.hpp
 *  @brief Persistent worker pool with a per block barrier.
 */

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace GPSOpenCl
{
/** @brief Runs one work item per index across persistent threads.
 *   Internal pipeline helper. run blocks until every item finished,
 *   so the caller sees a barrier. The caller thread drains items
 *   itself, and helper participation is capped to the remaining item
 *   count, so a two channel block synchronizes with one helper
 *   instead of the whole pool. Workers that wake without a free
 *   participation slot go straight back to sleep. Wakeups use
 *   notify_all: a targeted notify_one can be swallowed by a worker
 *   that already ran this block, deadlocking the barrier. Threads
 *   sleep between blocks and wake on a generation counter, claiming
 *   a bounded number of participation slots per run. Items are
 *   pulled from a shared atomic cursor,
 *   which balances uneven channel cost. With one worker or one item
 *   run executes inline on the caller thread. */
class TrackingWorkerPool
{
  public:
    /** @brief Work callback. Receives the item index. */
    using WorkFunction = std::function<void(int)>;

    /** @brief Start the pool.
     *  @param work       Called once per item index.
     *  @param maxWorkers Upper bound on thread count. */
    TrackingWorkerPool(WorkFunction work, int maxWorkers);

    /** @brief Signal shutdown and join every worker thread. */
    ~TrackingWorkerPool();
    TrackingWorkerPool(const TrackingWorkerPool &) = delete;
    TrackingWorkerPool &operator=(const TrackingWorkerPool &) = delete;
    TrackingWorkerPool(TrackingWorkerPool &&) = delete;
    TrackingWorkerPool &operator=(TrackingWorkerPool &&) = delete;

    /** @brief Process items 0 to itemCount minus one. Returns after all finish.
     *  @param itemCount Number of items this block. */
    void run(int itemCount);

    /** @brief Slowest worker wall time of the last run in milliseconds.
     *  @return Worst worker duration in ms. */
    double maxWorkerDurationMs() const;

  private:
    /** @brief Worker thread body. Waits for a new generation then drains the cursor.
     *  @param workerIndex Index of this worker into m_workers/m_workerDurationMs. */
    void workerLoop(int workerIndex);

    /** @brief Pull item indices from the cursor until none remain. */
    void drainCursor();

    WorkFunction m_work;                       ///< Per item callback.
    int m_numWorkers{1};                       ///< Worker thread count.
    std::vector<std::thread> m_workers;        ///< Worker threads.
    std::vector<double> m_workerDurationMs;    ///< Per worker wall time of the last run in ms.
    std::atomic<int> m_cursor{0};              ///< Next item index to claim.
    int m_itemCount{0};                        ///< Item count of the current run.
    std::mutex m_mutex;                        ///< Protects generation and pending count.
    std::condition_variable m_startCv;         ///< Wakes workers for a new run.
    std::condition_variable m_doneCv;          ///< Wakes the caller when the run finishes.
    uint64_t m_generation{0};                  ///< Bumped once per run.
    int m_pendingWorkers{0};                   ///< Helpers still busy this run.
    int m_helperTarget{0};                     ///< Helper participation slots this run.
    int m_claimedWorkers{0};                   ///< Helper slots already claimed this run.
    double m_callerDurationMs{0.0};            ///< Caller-thread drain wall time of the last run in ms.
    bool m_shutdown{false};                    ///< Set once in the destructor.
};
}

#endif
