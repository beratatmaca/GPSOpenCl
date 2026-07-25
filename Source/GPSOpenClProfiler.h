#ifndef INCLUDED_GPSOPENCL_PROFILER_H
#define INCLUDED_GPSOPENCL_PROFILER_H

#include "GPSOpenClSink.h"
#include "GPSOpenClStructs.h"
#include <chrono>
#include <memory>
#include <string>

namespace GPSOpenCl
{
class Profiler
{
  public:
    Profiler();
    ~Profiler();

    void setEnabled(bool enabled) { m_enabled = enabled; }
    bool isEnabled() const { return m_enabled; }

    void startBlock(uint32_t blockIndex, double timestamp);
    void recordStageTimeMs(const std::string &stageName, double timeMs);
    ProfilerOutput finishBlock();

    void setSink(std::shared_ptr<Sink> sink) { m_sink = sink; }

    class ScopedTimer
    {
      public:
        ScopedTimer(Profiler &profiler, const std::string &stageName)
            : m_profiler(profiler), m_stageName(stageName), m_start(std::chrono::high_resolution_clock::now())
        {
        }

        ~ScopedTimer()
        {
            if (m_profiler.isEnabled())
            {
                auto end = std::chrono::high_resolution_clock::now();
                double durationMs = std::chrono::duration<double, std::milli>(end - m_start).count();
                m_profiler.recordStageTimeMs(m_stageName, durationMs);
            }
        }

      private:
        Profiler &m_profiler;
        std::string m_stageName;
        std::chrono::high_resolution_clock::time_point m_start;
    };

  private:
    bool m_enabled{true};
    ProfilerOutput m_currentOutput{};
    std::chrono::high_resolution_clock::time_point m_blockStart{};
    std::shared_ptr<Sink> m_sink{nullptr};
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_PROFILER_H
