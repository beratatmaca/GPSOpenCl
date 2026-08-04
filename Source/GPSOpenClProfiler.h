#ifndef INCLUDED_GPSOPENCL_PROFILER_H
#define INCLUDED_GPSOPENCL_PROFILER_H

/** @file GPSOpenClProfiler.h
 *  @brief Per-block processing time profiler.
 */

#include "GPSOpenClSink.h"
#include "GPSOpenClStructs.h"
#include <chrono>
#include <cstdint>
#include <memory>
#include <utility>

namespace GPSOpenCl
{
/** @brief Per-block, per-stage processing time profiler. */
class Profiler
{
  public:
    /** @brief Pipeline stage identifier, used instead of string keys so recording a stage timing
     *   costs no allocation or string comparison. */
    enum class Stage : std::uint8_t
    {
        Acquisition,    ///< Acquisition correlate stage.
        Tracking,       ///< Tracking stage.
        NavDecode,      ///< Navigation decode stage.
        PvtSolve        ///< PVT solve stage.
    };

    Profiler();
    ~Profiler();
    Profiler(const Profiler &) = delete;
    Profiler &operator=(const Profiler &) = delete;
    Profiler(Profiler &&) = delete;
    Profiler &operator=(Profiler &&) = delete;

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
     *  @param stage  Stage identifier.
     *  @param timeMs Stage duration (ms). */
    void recordStageTimeMs(Stage stage, double timeMs);

    /** @brief Record tracking sub-stage timings for this block.
     *  @param earlyLatePromptGenMs Aggregate earlyLatePromptGen time across active channels (ms).
     *  @param numericOscillatorMs  Aggregate numericOscillator time across active channels (ms).
     *  @param accumulatorMs        Aggregate correlator-accumulation time across active channels (ms).
     *  @param maxWorkerMs          Slowest tracking worker's own wall-clock time this block (ms). */
    void recordTrackingSubStageTimings(double earlyLatePromptGenMs,
                                       double numericOscillatorMs,
                                       double accumulatorMs,
                                       double maxWorkerMs);

    /** @brief Finish block and return output.
     *  @return Profiler output struct. */
    ProfilerOutput finishBlock();

    /** @brief Set telemetry sink.
     *  @param sink Sink implementation. */
    void setSink(std::shared_ptr<Sink> sink) { m_sink = std::move(sink); }

    /** @brief RAII timer that records stage duration on destruction. Takes no clock sample at all
     *   when the profiler is disabled, so a disabled profiler adds no overhead to the timed region. */
    class ScopedTimer
    {
      public:
        /** @brief Start timing a stage.
         *  @param profiler Parent profiler.
         *  @param stage    Stage identifier. */
        ScopedTimer(Profiler &profiler, Stage stage) : m_profiler(profiler), m_stage(stage)
        {
            if (m_profiler.isEnabled())
            {
                m_start = std::chrono::high_resolution_clock::now();
                m_armed = true;
            }
        }

        ~ScopedTimer()
        {
            if (m_armed && m_profiler.isEnabled())
            {
                auto end = std::chrono::high_resolution_clock::now();
                const double durationMs = std::chrono::duration<double, std::milli>(end - m_start).count();
                m_profiler.recordStageTimeMs(m_stage, durationMs);
            }
        }

        ScopedTimer(const ScopedTimer &) = delete;
        ScopedTimer &operator=(const ScopedTimer &) = delete;
        ScopedTimer(ScopedTimer &&) = delete;
        ScopedTimer &operator=(ScopedTimer &&) = delete;

      private:
        Profiler &m_profiler;                                      ///< Parent profiler.
        Stage m_stage;                                             ///< Stage identifier.
        bool m_armed{false};                                       ///< True if a start sample was taken.
        std::chrono::high_resolution_clock::time_point m_start;    ///< Start time.
    };

  private:
    bool m_enabled{true};                                           ///< Profiling enabled flag.
    ProfilerOutput m_currentOutput{};                               ///< Current block output.
    std::chrono::high_resolution_clock::time_point m_blockStart;    ///< Block start time.
    std::shared_ptr<Sink> m_sink{nullptr};                          ///< Telemetry sink.
};
}

#endif
