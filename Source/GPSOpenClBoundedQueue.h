#ifndef INCLUDED_GPSOPENCL_BOUNDEDQUEUE_H
#define INCLUDED_GPSOPENCL_BOUNDEDQUEUE_H

#include <condition_variable>
#include <mutex>
#include <queue>

namespace GPSOpenCl
{
template <typename T>
class BoundedQueue
{
  public:
    explicit BoundedQueue(size_t maxCapacity = 16) : m_maxCapacity(maxCapacity), m_finished(false) {}

    bool push(T item)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cvPush.wait(lock, [this]() { return m_queue.size() < m_maxCapacity || m_finished; });
        if (m_finished) return false;
        m_queue.push(std::move(item));
        m_cvPop.notify_one();
        return true;
    }

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

    void finish()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_finished = true;
        m_cvPush.notify_all();
        m_cvPop.notify_all();
    }

  private:
    size_t m_maxCapacity;
    std::queue<T> m_queue;
    std::mutex m_mutex;
    std::condition_variable m_cvPush;
    std::condition_variable m_cvPop;
    bool m_finished;
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_BOUNDEDQUEUE_H
