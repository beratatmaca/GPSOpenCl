#ifndef INCLUDED_GPSOPENCL_APPLICATION_H
#define INCLUDED_GPSOPENCL_APPLICATION_H

/** @file GPSOpenClApplication.h
 *  @brief Main receiver processing pipeline.
 */

#include "GPSOpenClAcquisition.h"
#include "GPSOpenClAtmosphericCorrections.h"
#include "GPSOpenClChannel.h"
#include "GPSOpenClCode.h"
#include "GPSOpenClGPUCompute.h"
#include "GPSOpenClNmeaGenerator.h"
#include "GPSOpenClNavigationDecoder.h"
#include "GPSOpenClPVTSolver.h"
#include "GPSOpenClProfiler.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClSink.h"
#include "GPSOpenClSource.h"
#include "GPSOpenClStructs.h"
#include "GPSOpenClTracking.h"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace GPSOpenCl
{
/** @brief Main GPS receiver pipeline connecting all processing modules. */
class Application
{
  public:
    /** @brief Construct from configuration.
     *  @param conf Application configuration. */
    Application(Settings::Configuration conf);
    ~Application();

    /** @brief Run acquisition on all 32 PRNs.
     *  @param input IQ samples for one code period. */
    void searchForSatellites(const ComplexFloatVector &input);

    /** @brief Track all acquired satellites.
     *  @param input IQ samples for one code period. */
    void trackSatellites(const ComplexFloatVector &input);

    /** @brief Compute PVT solution from tracked satellites.
     *  @param solution Output position solution.
     *  @return True if valid solution. */
    bool computeNavigationSolution(ReceiverPvtSolution &solution);

    /** @brief Export telemetry to JSON file.
     *  @param filepath   Output file path.
     *  @param solution   PVT solution to export.
     *  @param utcTimeSec Receiver GPS time of week (s), used for the NMEA/timestamp fields. */
    void exportTelemetryJson(const std::string &filepath, const ReceiverPvtSolution &solution, double utcTimeSec);

    /** @brief Set telemetry sink.
     *  @param sink Sink implementation. */
    void setSink(std::shared_ptr<Sink> sink);

    /** @brief Set sample source.
     *  @param source Source implementation. */
    void setSource(std::shared_ptr<Source> source);

    /** @brief Get profiler reference.
     *  @return Profiler instance. */
    Profiler &getProfiler() { return m_profiler; }

    /** @brief Process one block through the full pipeline.
     *  @param input      IQ samples.
     *  @param blockIndex Current block index. */
    void processBlock(const ComplexFloatVector &input, uint32_t blockIndex);

  private:
    /** @brief Initialize all 32 satellite channels. */
    void initializeChannels();

    /** @brief Decode navigation bits for all confirmed-tracking channels. Must run every block
     *   regardless of the PVT fix-output cadence, since nav-bit decode needs to consume newly
     *   arrived Prompt samples continuously. */
    void updateChannelNavigation();

    /** @brief Track channels in [startIdx, endIdx). Used as the per-worker unit of the tracking pool.
     *  @param input    IQ samples.
     *  @param startIdx First channel index (inclusive).
     *  @param endIdx   Last channel index (exclusive). */
    void trackChannelRange(const ComplexFloatVector &input, int startIdx, int endIdx);

    /** @brief Persistent worker thread body: waits for a tracking dispatch, processes its channel
     *   range, and reports completion. Avoids spawning threads on every block.
     *  @param workerIndex Index of this worker, used to derive its channel range. */
    void workerLoop(int workerIndex);

    /** @brief Start the persistent tracking thread pool. */
    void startWorkerPool();

    /** @brief Signal and join all worker threads. */
    void stopWorkerPool();

    Acquisition *m_acquisition;              ///< Acquisition engine.
    Tracking *m_tracking;                    ///< Tracking engine.
    Settings::Configuration m_configuration; ///< Application configuration.

    Code *m_code;                            ///< C/A code generator.
    Compute *m_gpu;                          ///< GPU/CPU compute back-end.
    PVTSolver m_pvtSolver;                   ///< Position solver.
    NavigationDecoder m_navDecoder;           ///< Navigation decoder.
    NmeaGenerator m_nmeaGenerator;           ///< NMEA sentence generator.
    Channel m_channels[GPS_CA_SV_COUNT];     ///< Per-satellite channels.

    std::shared_ptr<Sink> m_sink{nullptr};   ///< Telemetry sink.
    std::shared_ptr<Source> m_source{nullptr}; ///< Sample source.
    Profiler m_profiler;                     ///< Processing time profiler.

    std::vector<std::thread> m_workers;             ///< Persistent tracking worker threads.
    std::mutex m_poolMutex;                         ///< Guards the pool dispatch state below.
    std::condition_variable m_startCv;              ///< Signals workers that a new range is ready.
    std::condition_variable m_doneCv;                ///< Signals the dispatcher that all workers finished.
    const ComplexFloatVector *m_currentTrackInput{nullptr}; ///< Block being tracked by the current dispatch.
    int m_generation{0};                            ///< Incremented on each new dispatch; workers compare against their last-seen value.
    int m_pendingWorkers{0};                        ///< Workers still processing the current dispatch.
    bool m_shutdownWorkers{false};                  ///< Set to stop all workers during destruction.
    int m_numWorkers{0};                            ///< Active worker count (0 or 1 disables the pool).
};
}

#endif
