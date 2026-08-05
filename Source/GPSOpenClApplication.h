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

    /** @brief Compute PVT solution from tracked satellites. Skips channels with anchors older than
     *   15 s. A stale anchor accumulates pseudorange error.
     *  @param solution Output position solution.
     *  @return True if valid solution. */
    bool computeNavigationSolution(ReceiverPvtSolution &solution);

    /** @brief Compute transmit time from the subframe anchor. Adds a sub-millisecond code-phase
     *   term. The subframe bit edge lands mid-block. Without it transmit times quantize to 1 ms.
     *   That is up to 300 km of error.
     *  @param subframeStartTow TOW at the subframe leading bit edge (s).
     *  @param elapsedSeconds   Whole code periods since the anchor block (s).
     *  @param driftChips       Accumulated code drift since the anchor block (chips).
     *  @param anchorChipsRaw   DLL code phase at the anchor block (chips, 0-1023).
     *  @return Satellite transmit time (s of week). */
    static double
        computeTransmitTime(double subframeStartTow, double elapsedSeconds, double driftChips, double anchorChipsRaw);

    /** @brief Resolve the C/A millisecond ambiguity. Transmit plus travel time must agree across
     *   satellites. A one-block decoder error shifts a whole millisecond. Snap such outliers onto
     *   the median cluster. The gate is 0.25 ms. Common shifts are absorbed by clock bias.
     *  @param transmitTimes   Per-satellite transmit times (s), corrected in place.
     *  @param impliedArrivals Per-satellite modeled arrival instants (s), same order. */
    static void snapTransmitTimesToMedianArrival(std::vector<double> &transmitTimes,
                                                 const std::vector<double> &impliedArrivals);

    /** @brief Reconstructed transmit time and pseudorange for one satellite. Captured by
     *   computeNavigationSolution(). For diagnostics and ground-truth checks. */
    struct PseudorangeSample
    {
        int svId{0};                              ///< Satellite vehicle ID.
        double transmitTimeSeconds{0.0};          ///< Reconstructed satellite transmit time (s).
        double measuredPseudorangeMeters{0.0};    ///< Reconstructed pseudorange (m).
    };

    /** @brief Get pseudorange samples from the last solve.
     *  @return One sample per contributing satellite. Empty below 4 ready satellites. */
    const std::vector<PseudorangeSample> &getLastPseudorangeSamples() const { return m_lastPseudorangeSamples; }

    /** @brief Per-channel bit-sync and subframe timing state. For diagnosing transmit-time
     *   reconstruction. Works without a PVT attempt. */
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

    /** @brief Get timing state for confirmed channels. Requires one decoded subframe of any ID.
     *   Full ephemeris is not required.
     *  @return One entry per such channel. */
    std::vector<ChannelDiagnostic> getChannelDiagnostics() const;

    /** @brief Queue a telemetry JSON export. Snapshots solution and channel state. The writer
     *   thread computes orbits and formats JSON.
     *  @param filepath    Output file path.
     *  @param solution    PVT solution to export.
     *  @param utcTimeSec  Receiver GPS time of week (s), used for the NMEA/timestamp fields.
     *  @param ggaSentence Pre-generated GGA sentence, reused instead of regenerating.
     *  @param rmcSentence Pre-generated RMC sentence, reused instead of regenerating.
     *  @param gsaSentence Pre-generated GSA sentence, reused instead of regenerating. */
    void exportTelemetryJson(const std::string &filepath,
                             const ReceiverPvtSolution &solution,
                             double utcTimeSec,
                             const std::string &ggaSentence,
                             const std::string &rmcSentence,
                             const std::string &gsaSentence);

    /** @brief Set telemetry sink.
     *  @param sink Sink implementation. */
    void setSink(const std::shared_ptr<Sink> &sink);

    /** @brief Set sample source.
     *  @param source Source implementation. */
    void setSource(std::shared_ptr<Source> source);

    /** @brief Process one block through the full pipeline.
     *  @param input      IQ samples.
     *  @param blockIndex Current block index. */
    void processBlock(const ComplexFloatVector &input, uint32_t blockIndex);

    /** @brief Per-satellite state captured for deferred telemetry JSON formatting. */
    struct SatelliteTelemetry
    {
        int prn{0};                  ///< Satellite PRN.
        bool acquired{false};        ///< True if the channel is acquired.
        float cn0{0.0f};             ///< Acquisition C/N0 estimate (dB-Hz).
        float doppler{0.0f};         ///< Acquired Doppler (Hz).
        bool computeOrbit{false};    ///< True if ephemeris and transmitTime are valid for az/el.
        GpsEphemeris ephemeris{};    ///< Decoded ephemeris for orbit computation.
        double transmitTime{0.0};    ///< Transmit time for orbit computation (s).
    };

    /** @brief Everything the telemetry JSON needs. Captured cheaply on the consumer thread. Orbit
     *   math and formatting run on the writer thread. */
    struct TelemetrySnapshot
    {
        ReceiverPvtSolution solution{};                ///< PVT solution.
        double utcTimeSec{0.0};                        ///< Receiver GPS time of week (s).
        std::vector<int> activePrns;                   ///< PRNs of acquired channels.
        std::vector<SatelliteTelemetry> satellites;    ///< Per-satellite state.
        std::string ggaSentence;                       ///< Pre-generated GGA sentence.
        std::string rmcSentence;                       ///< Pre-generated RMC sentence.
        std::string gsaSentence;                       ///< Pre-generated GSA sentence.
    };

    /** @brief Format a telemetry snapshot as JSON. Runs on the writer thread. Computes
     *   per-satellite orbits and azimuth elevation.
     *  @param snapshot Captured telemetry state.
     *  @return JSON document text. */
    static std::string formatTelemetryJson(const TelemetrySnapshot &snapshot);

  private:
    /** @brief One pending background acquisition search. Holds a sample snapshot and the channel to
     *   search. Handed to the acquisition worker thread. */
    struct AcquisitionJob
    {
        ComplexFloatVector input;    ///< Snapshot of the block IQ samples.
        int channelIndex{0};         ///< Channel index (0-based, PRN - 1) to search.
    };

    /** @brief Result of a finished background acquisition search. The consumer thread finalizes
     *   channel state from it. */
    struct AcquisitionResult
    {
        int channelIndex{0};                 ///< Channel index (0-based, PRN - 1) that was searched.
        double correlateMs{0.0};             ///< Wall-clock duration of the correlate() call (ms).
        ComplexFloatVector recycledInput;    ///< Job sample buffer, returned for allocation reuse.
    };

    /** @brief One pending asynchronous text write. Either a file overwrite or a console print. The
     *   consumer thread never blocks on I/O. Telemetry JSON is formatted on the writer thread. */
    struct AsyncOutputJob
    {
        bool isConsole{false};                           ///< True writes to stdout, false overwrites filePath.
        std::string filePath;                            ///< Target file path (unused if isConsole).
        std::string content;                             ///< Text to write.
        std::unique_ptr<TelemetrySnapshot> telemetry;    ///< Deferred telemetry snapshot, or null.
    };

    /** @brief Initialize all 32 satellite channels. */
    void initializeChannels();

    /** @brief Run acquisition on one channel. Applies the C/N0 threshold. Starts tracking on
     *   success. Does nothing if the channel is not eligible.
     *  @param input        IQ samples for one code period.
     *  @param channelIndex Channel index (0-based, PRN - 1). */
    void searchOneChannel(const ComplexFloatVector &input, int channelIndex);

    /** @brief Apply finished acquisition search results. Checks C/N0, starts tracking, publishes
     *   telemetry. Must run on the consumer thread only. It mutates shared channel lifecycle state.
     *  @param channelIndex Channel index (0-based, PRN - 1) whose search just completed. */
    void finalizeAcquisition(int channelIndex, double correlateMs);

    /** @brief Persistent background acquisition thread body. Runs the Doppler search off the
     *   consumer thread. The search takes tens of milliseconds. Tracking and PVT never stall on
     *   it. */
    void acquisitionWorkerLoop();

    /** @brief Persistent background writer thread body. Drains m_outputQueue and writes. Slow I/O
     *   never stalls the consumer thread. */
    void outputWriterLoop();

    /** @brief Decode navigation bits for confirmed channels. Must run every block. Decode consumes
     *   new Prompt samples continuously. */
    void updateChannelNavigation();

    /** @brief Publish TrackingOutput for every active channel. Runs after the tracking barrier.
     *   Publish cost never extends the barrier. */
    void publishTrackingOutputs();

    /** @brief Pull channel indices from the shared cursor. Track each until the list is exhausted.
     *   This load balances across workers.
     *  @param input IQ samples. */
    void trackFromCursor(const ComplexFloatVector &input);

    /** @brief Persistent worker thread body. Waits for a dispatch, tracks, reports completion.
     *   Avoids spawning threads per block.
     *  @param workerIndex Index of this worker, used for its duration telemetry slot. */
    void workerLoop(int workerIndex);

    /** @brief Start the persistent tracking thread pool. */
    void startWorkerPool();

    /** @brief Signal and join all worker threads. */
    void stopWorkerPool();

    std::unique_ptr<Acquisition> m_acquisition;                 ///< Acquisition engine.
    Tracking *m_tracking{nullptr};                              ///< Tracking engine.
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
    uint32_t m_currentBlockIndex{0};                            ///< Block index of the block being processed.

    std::vector<std::thread> m_workers;                         ///< Persistent tracking worker threads.
    std::mutex m_poolMutex;                                     ///< Guards the pool dispatch state below.
    std::condition_variable m_startCv;                          ///< Signals workers that a new range is ready.
    std::condition_variable m_doneCv;                           ///< Signals the dispatcher that all workers finished.
    const ComplexFloatVector *m_currentTrackInput{nullptr};     ///< Block being tracked by the current dispatch.
    std::vector<int> m_activeChannels;                          ///< Channel indices to track this dispatch.
    std::atomic<int> m_channelCursor{0};                        ///< Next m_activeChannels slot a worker should take.
    std::vector<double> m_workerDurationMs;    ///< Per-worker tracking time this block (ms), own index only.
    uint64_t m_generation{0};                  ///< Dispatch counter, workers compare their last seen value.
    int m_pendingWorkers{0};                   ///< Workers still processing the current dispatch.
    bool m_shutdownWorkers{false};             ///< Set to stop all workers during destruction.
    int m_numWorkers{0};                       ///< Active worker count (0 or 1 disables the pool).

    ComplexFloatVector m_acqInputPool;         ///< Recycled buffer for acquisition job snapshots.
    std::thread m_acquisitionThread;           ///< Background acquisition worker thread.
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
