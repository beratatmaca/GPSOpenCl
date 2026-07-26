#ifndef INCLUDED_GPSOPENCL_BOUNDEDQUEUE_H
#define INCLUDED_GPSOPENCL_BOUNDEDQUEUE_H

/** @file GPSOpenClBoundedQueue.h
 *  @brief Thread-safe bounded producer-consumer queue.
 */

#include <condition_variable>
#include <mutex>
#include <queue>

namespace GPSOpenCl
{
/** @brief Thread-safe bounded blocking queue.
 *  @tparam T Element type. */
template <typename T>
class BoundedQueue
{
  public:
    /** @brief Construct with max capacity.
     *  @param maxCapacity Maximum queue size. */
    explicit BoundedQueue(size_t maxCapacity = 16) : m_maxCapacity(maxCapacity), m_finished(false) {}

    /** @brief Push an item, blocking if full.
     *  @param item Item to push.
     *  @return False if queue is finished. */
    bool push(T item)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cvPush.wait(lock, [this]() { return m_queue.size() < m_maxCapacity || m_finished; });
        if (m_finished) return false;
        m_queue.push(std::move(item));
        m_cvPop.notify_one();
        return true;
    }

    /** @brief Push an item without blocking; drops it if the queue is full instead of waiting.
     *   For producers that must never stall on a slow consumer (e.g. telemetry callers on a
     *   real-time path), matching a "may drop" delivery contract rather than backpressure.
     *  @param item Item to push.
     *  @return True if enqueued; false if the queue was full or finished (item is dropped). */
    bool tryPush(T item)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_finished || m_queue.size() >= m_maxCapacity) return false;
        m_queue.push(std::move(item));
        m_cvPop.notify_one();
        return true;
    }

    /** @brief Pop an item, blocking if empty.
     *  @param item Output item.
     *  @return False if queue is empty and finished. */
    bool pop(T &item)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cvPop.wait(lock, [this]() { return !m_queue.empty() || m_finished; });
        if (m_queue.empty() && m_finished) return false;
        item = std::move(m_queue.front());
        m_queue.pop();
        m_cvPush.notify_one();
        return true;
    }

    /** @brief Signal that no more items will be pushed. */
    void finish()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_finished = true;
        m_cvPush.notify_all();
        m_cvPop.notify_all();
    }

  private:
    size_t m_maxCapacity;                ///< Maximum queue size.
    std::queue<T> m_queue;               ///< Internal queue.
    std::mutex m_mutex;                  ///< Queue mutex.
    std::condition_variable m_cvPush;    ///< Push condition variable.
    std::condition_variable m_cvPop;     ///< Pop condition variable.
    bool m_finished;                     ///< Finished flag.
};
}

#endif
