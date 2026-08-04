#include "GPSOpenClProfiler.h"

namespace GPSOpenCl
{
Profiler::Profiler()
{
    m_currentOutput.structVersion = STRUCT_VERSION_1;
}

Profiler::~Profiler() = default;

void Profiler::startBlock(uint32_t blockIndex, double timestamp)
{
    m_currentOutput = ProfilerOutput{};
    m_currentOutput.structVersion = STRUCT_VERSION_1;
    m_currentOutput.blockIndex = blockIndex;
    m_currentOutput.timestamp = timestamp;
    m_blockStart = std::chrono::high_resolution_clock::now();
}

void Profiler::recordStageTimeMs(Stage stage, double timeMs)
{
    switch (stage)
    {
        case Stage::Acquisition:
            m_currentOutput.acquisitionTimeMs = timeMs;
            break;
        case Stage::Tracking:
            m_currentOutput.trackingTimeMs = timeMs;
            break;
        case Stage::NavDecode:
            m_currentOutput.navDecodeTimeMs = timeMs;
            break;
        case Stage::PvtSolve:
            m_currentOutput.pvtSolveTimeMs = timeMs;
            break;
    }
}

void Profiler::recordTrackingSubStageTimings(double earlyLatePromptGenMs,
                                             double numericOscillatorMs,
                                             double accumulatorMs,
                                             double maxWorkerMs)
{
    m_currentOutput.earlyLatePromptGenTimeMs = earlyLatePromptGenMs;
    m_currentOutput.numericOscillatorTimeMs = numericOscillatorMs;
    m_currentOutput.accumulatorTimeMs = accumulatorMs;
    m_currentOutput.trackingMaxWorkerTimeMs = maxWorkerMs;
}

ProfilerOutput Profiler::finishBlock()
{
    auto end = std::chrono::high_resolution_clock::now();
    m_currentOutput.totalTimeMs = std::chrono::duration<double, std::milli>(end - m_blockStart).count();

    if (m_sink && m_enabled)
    {
        m_sink->publishProfilerOutput(m_currentOutput);
    }

    return m_currentOutput;
}
}
