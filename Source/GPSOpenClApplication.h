#ifndef INCLUDED_GPSOPENCL_APPLICATION_H
#define INCLUDED_GPSOPENCL_APPLICATION_H

/** @file GPSOpenClApplication.h
 *  @brief Main receiver processing pipeline.
 */

#include "GPSOpenClAcquisition.h"
#include "GPSOpenClAtmosphericCorrections.h"
#include "GPSOpenClBoundedQueue.h"
#include "GPSOpenClChannel.h"
#include "GPSOpenClCode.h"
#include "GPSOpenClGPUCompute.h"
#include "GPSOpenClNavigationDecoder.h"
#include "GPSOpenClNmeaGenerator.h"
#include "GPSOpenClPVTSolver.h"
#include "GPSOpenClProfiler.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClSink.h"
#include "GPSOpenClSource.h"
#include "GPSOpenClStructs.h"
#include "GPSOpenClTracking.h"

#include <atomic>
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
    Application(const Settings::Configuration &conf);
    ~Application();
    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;
    Application(Application &&) = delete;
    Application &operator=(Application &&) = delete;

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

    /** @brief One satellite's transmit-time/pseudorange reconstruction from the last
     *   computeNavigationSolution() call, for diagnostics and ground-truth verification. */
    struct PseudorangeSample
    {
        int svId{0};                              ///< Satellite vehicle ID.
        double transmitTimeSeconds{0.0};          ///< Reconstructed satellite transmit time (s).
        double measuredPseudorangeMeters{0.0};    ///< Reconstructed pseudorange (m).
    };

    /** @brief Get per-satellite transmit-time/pseudorange reconstruction from the most recent
     *   computeNavigationSolution() call.
     *  @return Samples, one per satellite that contributed to that call (empty if fewer than 4
     *   satellites were ready). */
    const std::vector<PseudorangeSample> &getLastPseudorangeSamples() const { return m_lastPseudorangeSamples; }

    /** @brief Per-channel bit-sync/subframe-timing state, for diagnosing transmit-time reconstruction
     *   independently of whether enough satellites are ready for a PVT attempt. */
    struct ChannelDiagnostic
    {
        int svId{0};                             ///< Satellite vehicle ID.
        int bitSyncPhase{-1};                    ///< Locked sample-level bit-edge phase, 0-19.
        double subframeStartTow{0.0};            ///< TOW of last decoded subframe (s).
        size_t subframeStartSample{0};           ///< Sample index of last decoded subframe.
        float codePhaseAtSubframeStart{0.0f};    ///< DLL code phase (chips) at that sample.
        double elapsedSeconds{0.0};              ///< (promptCount - subframeStartSample) * code period (s).
        double candidateNowTow{0.0};             ///< subframeStartTow + elapsedSeconds (s).
        float codePhaseNow{0.0f};                ///< DLL code phase (chips) at the current (latest) sample.
    };

    /** @brief Get bit-sync/subframe-timing state for every tracking-confirmed channel that has
     *   decoded at least one subframe (any subframe ID -- full ephemeris not required).
     *  @return One entry per such channel. */
    std::vector<ChannelDiagnostic> getChannelDiagnostics() const;

    /** @brief Export telemetry to JSON file.
     *  @param filepath   Output file path.
     *  @param solution   PVT solution to export.
     *  @param utcTimeSec Receiver GPS time of week (s), used for the NMEA/timestamp fields. */
    void exportTelemetryJson(const std::string &filepath, const ReceiverPvtSolution &solution, double utcTimeSec);

    /** @brief Set telemetry sink.
     *  @param sink Sink implementation. */
    void setSink(const std::shared_ptr<Sink> &sink);

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
    /** @brief One pending background acquisition search: a snapshot of the block's samples plus the
     *   channel to search, handed off to the acquisition worker thread. */
    struct AcquisitionJob
    {
        ComplexFloatVector input;    ///< Snapshot of the block's IQ samples.
        int channelIndex{0};         ///< Channel index (0-based, PRN - 1) to search.
    };

    /** @brief Result of a completed background acquisition search, handed back to the consumer
     *   thread for channel-state finalization. */
    struct AcquisitionResult
    {
        int channelIndex{0};        ///< Channel index (0-based, PRN - 1) that was searched.
        double correlateMs{0.0};    ///< Wall-clock duration of the correlate() call (ms).
    };

    /** @brief One pending asynchronous text write: either a full file overwrite or a console print,
     *   so the consumer thread never blocks on disk or stdout I/O. */
    struct AsyncOutputJob
    {
        bool isConsole{false};    ///< True: write content to stdout. False: overwrite filePath.
        std::string filePath;     ///< Target file path (unused if isConsole).
        std::string content;      ///< Text to write.
    };

    /** @brief Initialize all 32 satellite channels. */
    void initializeChannels();

    /** @brief Run acquisition on a single channel, applying the C/N0 threshold and starting tracking
     *   on success. No-op if the channel isn't currently eligible for acquisition.
     *  @param input        IQ samples for one code period.
     *  @param channelIndex Channel index (0-based, PRN - 1). */
    void searchOneChannel(const ComplexFloatVector &input, int channelIndex);

    /** @brief Apply a completed acquisition search's results: C/N0 threshold check, tracking
     *   initialization on success, and telemetry publish. Must run on the consumer thread only,
     *   since it mutates channel lifecycle state that the tracking worker pool also touches.
     *  @param channelIndex Channel index (0-based, PRN - 1) whose search just completed. */
    void finalizeAcquisition(int channelIndex);

    /** @brief Persistent background acquisition thread body: waits for a job, runs the correlation
     *   search, and reports completion. Keeps the ~tens-of-ms Doppler search off the consumer
     *   thread so it never stalls tracking/nav-decode/PVT for the current block. */
    void acquisitionWorkerLoop();

    /** @brief Persistent background writer thread body: drains m_outputQueue and performs the
     *   actual file/console write, so a slow disk or stdout never stalls the consumer thread. */
    void outputWriterLoop();

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

    std::unique_ptr<Acquisition> m_acquisition;                 ///< Acquisition engine.
    Tracking *m_tracking;                                       ///< Tracking engine.
    Settings::Configuration m_configuration;                    ///< Application configuration.

    std::unique_ptr<Code> m_code;                               ///< C/A code generator.
    std::unique_ptr<Compute> m_gpu;                             ///< GPU/CPU compute back-end.
    PVTSolver m_pvtSolver;                                      ///< Position solver.
    NavigationDecoder m_navDecoder;                             ///< Navigation decoder.
    NmeaGenerator m_nmeaGenerator;                              ///< NMEA sentence generator.
    Channel m_channels[GPS_CA_SV_COUNT];                        ///< Per-satellite channels.
    std::vector<PseudorangeSample> m_lastPseudorangeSamples;    ///< Set by the last computeNavigationSolution() call.

    std::shared_ptr<Sink> m_sink{nullptr};                      ///< Telemetry sink.
    std::shared_ptr<Source> m_source{nullptr};                  ///< Sample source.
    Profiler m_profiler;                                        ///< Processing time profiler.

    std::vector<std::thread> m_workers;                         ///< Persistent tracking worker threads.
    std::mutex m_poolMutex;                                     ///< Guards the pool dispatch state below.
    std::condition_variable m_startCv;                          ///< Signals workers that a new range is ready.
    std::condition_variable m_doneCv;                           ///< Signals the dispatcher that all workers finished.
    const ComplexFloatVector *m_currentTrackInput{nullptr};     ///< Block being tracked by the current dispatch.
    std::vector<double> m_workerDurationMs;    ///< Per-worker wall-clock time in trackChannelRange this
                                               ///< block (ms); each worker writes only its own index.
    int m_generation{0};        ///< Incremented on each new dispatch; workers compare against their last-seen value.
    int m_pendingWorkers{0};    ///< Workers still processing the current dispatch.
    bool m_shutdownWorkers{false};                                  ///< Set to stop all workers during destruction.
    int m_numWorkers{0};                                            ///< Active worker count (0 or 1 disables the pool).

    std::thread m_acquisitionThread;                                ///< Background acquisition worker thread.
    BoundedQueue<AcquisitionJob> m_acquisitionJobQueue{1};          ///< Consumer-to-worker job handoff (one in flight).
    BoundedQueue<AcquisitionResult> m_acquisitionResultQueue{4};    ///< Worker-to-consumer result handoff.
    std::atomic<bool> m_acquisitionBusy{false};                     ///< True while a background search is in flight.
    int m_nextAcquisitionChannel{0};                                ///< Cursor for the cold-start sweep.
    bool m_coldStartSweepDone{false};                               ///< True once every channel has been searched once.

    std::thread m_outputWriterThread;                  ///< Background telemetry JSON/console writer thread.
    BoundedQueue<AsyncOutputJob> m_outputQueue{32};    ///< Handoff queue for async file/console writes.
};
}

#endif
