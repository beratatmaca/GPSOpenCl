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

    /** @brief Reconstruct a satellite's transmit time from its subframe anchor. The sub-millisecond
     *   anchor term exists because the subframe's leading bit edge does not fall on a receiver
     *   block boundary: the decoder's edge refinement guarantees the anchor block contains the
     *   edge, which arrives (1023 - anchorChipsRaw) chips after that block starts. Without it every
     *   satellite's transmit time is quantized to the 1 ms block grid, which puts up to 1 ms
     *   (300 km) of per-satellite error on the pseudoranges.
     *  @param subframeStartTow TOW at the subframe's leading bit edge (s).
     *  @param elapsedSeconds   Whole code periods elapsed since the anchor block (s).
     *  @param driftChips       Accumulated code-frequency drift since the anchor block (chips).
     *  @param anchorChipsRaw   DLL code phase at the anchor block (chips, 0-1023).
     *  @return Satellite transmit time (s of week). */
    static double
        computeTransmitTime(double subframeStartTow, double elapsedSeconds, double driftChips, double anchorChipsRaw);

    /** @brief C/A millisecond-ambiguity resolution. Every satellite received the same epoch, so
     *   each transmit time plus its modeled travel time (impliedArrivals) must agree across the
     *   constellation to well under a millisecond; a bit edge attributed one block off in the
     *   decoder shifts a satellite's transmit time by an integer millisecond. Snaps every satellite
     *   whose offset from the constellation median sits within 0.25 ms of a whole millisecond back
     *   onto the median cluster. The median cluster does not need to hold the true epoch: a common
     *   whole-millisecond shift across all satellites is absorbed by the receiver clock bias in the
     *   WLS solve, so cross-satellite consistency is what matters, not absolute truth.
     *  @param transmitTimes   Per-satellite transmit times (s), corrected in place.
     *  @param impliedArrivals Per-satellite modeled arrival instants (s), same order. */
    static void snapTransmitTimesToMedianArrival(std::vector<double> &transmitTimes,
                                                 const std::vector<double> &impliedArrivals);

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

    /** @brief Queue a telemetry JSON export: captures a snapshot of solution and channel state and
     *   hands it to the output writer thread, which computes satellite orbits and formats the JSON
     *   off the consumer thread.
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
        int channelIndex{0};                 ///< Channel index (0-based, PRN - 1) that was searched.
        double correlateMs{0.0};             ///< Wall-clock duration of the correlate() call (ms).
        ComplexFloatVector recycledInput;    ///< Job's sample buffer, returned so its allocation is reused.
    };

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

    /** @brief Snapshot of everything the telemetry JSON needs, captured cheaply on the consumer
     *   thread so orbit computation and string formatting run on the output writer thread. */
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

    /** @brief One pending asynchronous text write: either a full file overwrite or a console print,
     *   so the consumer thread never blocks on disk or stdout I/O. A job carrying a telemetry
     *   snapshot has its JSON formatted on the writer thread before the file write. */
    struct AsyncOutputJob
    {
        bool isConsole{false};                           ///< True: write content to stdout. False: overwrite filePath.
        std::string filePath;                            ///< Target file path (unused if isConsole).
        std::string content;                             ///< Text to write.
        std::unique_ptr<TelemetrySnapshot> telemetry;    ///< Deferred telemetry snapshot, or null.
    };

    /** @brief Format a telemetry snapshot as JSON. Runs on the output writer thread; performs the
     *   per-satellite orbit and azimuth/elevation computation.
     *  @param snapshot Captured telemetry state.
     *  @return JSON document text. */
    static std::string formatTelemetryJson(const TelemetrySnapshot &snapshot);

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
    void finalizeAcquisition(int channelIndex, double correlateMs);

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

    /** @brief Publish every active channel's TrackingOutput through the sink. Runs on the
     *   dispatching thread after the tracking barrier, so publish cost never extends the barrier. */
    void publishTrackingOutputs();

    /** @brief Pull channel indices from the shared dispatch cursor and track each one until the
     *   active-channel list is exhausted. Used as the per-worker unit of the tracking pool, so load
     *   balances across workers regardless of which PRNs are active.
     *  @param input IQ samples. */
    void trackFromCursor(const ComplexFloatVector &input);

    /** @brief Persistent worker thread body: waits for a tracking dispatch, pulls channels from the
     *   shared cursor, and reports completion. Avoids spawning threads on every block.
     *  @param workerIndex Index of this worker, used for its duration telemetry slot. */
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
    uint32_t m_currentBlockIndex{0};                            ///< Block index of the block being processed.

    std::vector<std::thread> m_workers;                         ///< Persistent tracking worker threads.
    std::mutex m_poolMutex;                                     ///< Guards the pool dispatch state below.
    std::condition_variable m_startCv;                          ///< Signals workers that a new range is ready.
    std::condition_variable m_doneCv;                           ///< Signals the dispatcher that all workers finished.
    const ComplexFloatVector *m_currentTrackInput{nullptr};     ///< Block being tracked by the current dispatch.
    std::vector<int> m_activeChannels;                          ///< Channel indices to track this dispatch.
    std::atomic<int> m_channelCursor{0};                        ///< Next m_activeChannels slot a worker should take.
    std::vector<double> m_workerDurationMs;    ///< Per-worker wall-clock time in trackChannelRange this
                                               ///< block (ms); each worker writes only its own index.
    uint64_t m_generation{0};    ///< Incremented on each new dispatch; workers compare against their last-seen value.
    int m_pendingWorkers{0};    ///< Workers still processing the current dispatch.
    bool m_shutdownWorkers{false};                                  ///< Set to stop all workers during destruction.
    int m_numWorkers{0};                                            ///< Active worker count (0 or 1 disables the pool).

    ComplexFloatVector m_acqInputPool;                              ///< Recycled buffer for acquisition job snapshots.
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
