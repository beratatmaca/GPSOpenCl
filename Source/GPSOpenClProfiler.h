#ifndef INCLUDED_GPSOPENCL_PROFILER_H
#define INCLUDED_GPSOPENCL_PROFILER_H

/** @file GPSOpenClProfiler.h
 *  @brief Per-block processing time profiler.
 */

#include "GPSOpenClSink.h"
#include "GPSOpenClStructs.h"
#include <chrono>
#include <memory>
#include <string>

namespace GPSOpenCl
{
/** @brief Per-block, per-stage processing time profiler. */
class Profiler
{
  public:
    Profiler();
    ~Profiler();

    /** @brief Enable or disable profiling.
     *  @param enabled True to enable. */
    void setEnabled(bool enabled) { m_enabled = enabled; }

    /** @brief Check if profiling is enabled.
     *  @return True if enabled. */
    bool isEnabled() const { return m_enabled; }

    /** @brief Start timing a new block.
     *  @param blockIndex Block index.
     *  @param timestamp  Block timestamp (s). */
    void startBlock(uint32_t blockIndex, double timestamp);

    /** @brief Record a stage timing.
     *  @param stageName Stage name.
     *  @param timeMs    Stage duration (ms). */
    void recordStageTimeMs(const std::string &stageName, double timeMs);

    /** @brief Record tracking sub-stage timings for this block.
     *  @param earlyLatePromptGenMs Aggregate earlyLatePromptGen time across active channels (ms).
     *  @param numericOscillatorMs  Aggregate numericOscillator time across active channels (ms).
     *  @param accumulatorMs        Aggregate correlator-accumulation time across active channels (ms).
     *  @param maxWorkerMs          Slowest tracking worker's own wall-clock time this block (ms). */
    void recordTrackingSubStageTimings(double earlyLatePromptGenMs, double numericOscillatorMs, double accumulatorMs,
                                       double maxWorkerMs);

    /** @brief Finish block and return output.
     *  @return Profiler output struct. */
    ProfilerOutput finishBlock();

    /** @brief Set telemetry sink.
     *  @param sink Sink implementation. */
    void setSink(std::shared_ptr<Sink> sink) { m_sink = sink; }

    /** @brief RAII timer that records stage duration on destruction. */
    class ScopedTimer
    {
      public:
        /** @brief Start timing a stage.
         *  @param profiler  Parent profiler.
         *  @param stageName Stage name. */
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
        Profiler &m_profiler;                                       ///< Parent profiler.
        std::string m_stageName;                                    ///< Stage name.
        std::chrono::high_resolution_clock::time_point m_start;     ///< Start time.
    };

  private:
    bool m_enabled{true};                                           ///< Profiling enabled flag.
    ProfilerOutput m_currentOutput{};                               ///< Current block output.
    std::chrono::high_resolution_clock::time_point m_blockStart{};  ///< Block start time.
    std::shared_ptr<Sink> m_sink{nullptr};                          ///< Telemetry sink.
};
}

#endif
