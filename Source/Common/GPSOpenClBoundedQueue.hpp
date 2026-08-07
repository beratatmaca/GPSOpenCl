#ifndef INCLUDED_GPSOPENCL_BOUNDEDQUEUE_HPP
#define INCLUDED_GPSOPENCL_BOUNDEDQUEUE_HPP

/** @file GPSOpenClBoundedQueue.hpp
 *  @brief Thread-safe bounded producer-consumer queue.
 */

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <utility>
#include <vector>

namespace GPSOpenCl
{
/** @brief Thread safe bounded blocking queue over a fixed ring. Storage is allocated once at
 *   construction. Steady state push and pop never allocate.
 *  @tparam T Element type, must be default constructible and movable. */
template<typename T> class BoundedQueue
{
  public:
    /** @brief Construct with max capacity.
     *  @param maxCapacity Maximum queue size. */
    explicit BoundedQueue(size_t maxCapacity = 16)
        : m_storage(maxCapacity == 0 ? 1 : maxCapacity),
          m_maxCapacity(maxCapacity == 0 ? 1 : maxCapacity)
    {
    }

    /** @brief Push an item, blocking if full.
     *  @param item Item to push.
     *  @return False if queue is finished. */
    bool push(T item)
    {
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cvPush.wait(lock, [this]() { return m_count < m_maxCapacity || m_finished; });
            if (m_finished)
            {
                return false;
            }
            enqueueLocked(std::move(item));
        }
        m_cvPop.notify_one();
        return true;
    }

    /** @brief Push without blocking. A full queue drops the item. For producers that must never
     *   stall on a slow consumer. Matches a may drop delivery contract.
     *  @param item Item to push.
     *  @return True if enqueued. False when full or finished, item dropped. */
    bool tryPush(T item)
    {
        {
            const std::lock_guard<std::mutex> lock(m_mutex);
            if (m_finished || m_count >= m_maxCapacity)
            {
                return false;
            }
            enqueueLocked(std::move(item));
        }
        m_cvPop.notify_one();
        return true;
    }

    /** @brief Pop an item, blocking if empty.
     *  @param item Output item.
     *  @return False if queue is empty and finished. */
    bool pop(T &item)
    {
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cvPop.wait(lock, [this]() { return m_count > 0 || m_finished; });
            if (m_count == 0 && m_finished)
            {
                return false;
            }
            dequeueLocked(item);
        }
        m_cvPush.notify_one();
        return true;
    }

    /** @brief Pop without blocking. An empty queue returns false at once. For consumers that must
     *   never stall waiting on a producer.
     *  @param item Output item.
     *  @return True if an item was popped. False when the queue was empty. */
    bool tryPop(T &item)
    {
        {
            const std::lock_guard<std::mutex> lock(m_mutex);
            if (m_count == 0)
            {
                return false;
            }
            dequeueLocked(item);
        }
        m_cvPush.notify_one();
        return true;
    }

    /** @brief Signal that no more items will be pushed. */
    void finish()
    {
        const std::unique_lock<std::mutex> lock(m_mutex);
        m_finished = true;
        m_cvPush.notify_all();
        m_cvPop.notify_all();
    }

  private:
    /** @brief Append an item at the ring tail. Caller holds the mutex.
     *  @param item Item to append. */
    void enqueueLocked(T &&item)
    {
        m_storage[(m_head + m_count) % m_maxCapacity] = std::move(item);
        m_count++;
    }

    /** @brief Remove the item at the ring head. Caller holds the mutex.
     *  @param item Output item. */
    void dequeueLocked(T &item)
    {
        item = std::move(m_storage[m_head]);
        m_head = (m_head + 1) % m_maxCapacity;
        m_count--;
    }

    std::vector<T> m_storage;            ///< Fixed ring storage.
    size_t m_maxCapacity;                ///< Maximum queue size.
    size_t m_head{0};                    ///< Index of the oldest element.
    size_t m_count{0};                   ///< Number of queued elements.
    std::mutex m_mutex;                  ///< Queue mutex.
    std::condition_variable m_cvPush;    ///< Push condition variable.
    std::condition_variable m_cvPop;     ///< Pop condition variable.
    bool m_finished{false};              ///< Finished flag.
};
}

#endif
