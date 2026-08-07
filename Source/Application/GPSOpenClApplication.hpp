#ifndef INCLUDED_GPSOPENCL_APPLICATION_HPP
#define INCLUDED_GPSOPENCL_APPLICATION_HPP

/** @file GPSOpenClApplication.hpp
 *  @brief Main receiver processing pipeline.
 */

#include "Acquisition/GPSOpenClAcquisition.hpp"
#include "Acquisition/GPSOpenClCaCodeGenerator.hpp"
#include "Common/GPSOpenClBoundedQueue.hpp"
#include "Common/GPSOpenClProfiler.hpp"
#include "Common/GPSOpenClSettings.hpp"
#include "Common/GPSOpenClStructs.hpp"
#include "Gpu/GPSOpenClSpectrumEngine.hpp"
#include "Input/GPSOpenClSource.hpp"
#include "NavDecode/GPSOpenClNavigationDecoder.hpp"
#include "Pvt/GPSOpenClMeasurementAssembler.hpp"
#include "Pvt/GPSOpenClPVTSolver.hpp"
#include "Sink/GPSOpenClNmeaGenerator.hpp"
#include "Sink/GPSOpenClSink.hpp"
#include "Sink/GPSOpenClTelemetryExporter.hpp"
#include "Tracking/GPSOpenClChannel.hpp"
#include "Tracking/GPSOpenClTracking.hpp"
#include "Tracking/GPSOpenClTrackingWorkerPool.hpp"

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

    /** @brief Stop background acquisition and worker threads. */
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

    /** @brief Process one block through the full pipeline.
     *  @param input      IQ samples.
     *  @param blockIndex Current block index. */
    void processBlock(const ComplexFloatVector &input, uint32_t blockIndex);

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

    /** @brief Initialize all 32 satellite channels. */
    void initializeChannels();

    /** @brief Run acquisition on one channel. Applies the C/N0 threshold. Starts tracking on
     *   success. Does nothing if the channel is not eligible.
     *  @param input        IQ samples for one code period.
     *  @param channelIndex Channel index (0-based, PRN - 1). */
    void searchOneChannel(const ComplexFloatVector &input, int channelIndex);

    /** @brief Track one active channel slot. Worker pool callback.
     *  @param slot Index into m_activeChannels. */
    void trackOneActiveChannel(int slot);

    /** @brief Apply finished acquisition search results. Checks C/N0, starts tracking, publishes
     *   telemetry. Must run on the consumer thread only. It mutates shared channel lifecycle state.
     *  @param channelIndex Channel index (0-based, PRN - 1) whose search just completed.
     *  @param correlateMs  Correlation search time (ms), for telemetry. */
    void finalizeAcquisition(int channelIndex, double correlateMs);

    /** @brief Persistent background acquisition thread body. Runs the Doppler search off the
     *   consumer thread. The search takes tens of milliseconds. Tracking and PVT never stall on
     *   it. */
    void acquisitionWorkerLoop();

    /** @brief Decode navigation bits for confirmed channels. Must run every block. Decode consumes
     *   new Prompt samples continuously. */
    void updateChannelNavigation();

    /** @brief Publish TrackingOutput for every active channel. Runs after the tracking barrier.
     *   Publish cost never extends the barrier. */
    void publishTrackingOutputs();

    /** @brief Whether the solver reference position can steer millisecond snapping. The compiled-in
     *   seed is trusted optimistically so a cold start near it resolves the systematic one-code-
     *   period bit-edge ambiguity. Repeated pre-fix solve failures are the signature of a wrong
     *   seed, so trust is withdrawn until the first successful fix.
     *  @return True while the seed is presumed good or after any successful fix. */
    bool isReferencePositionTrusted() const { return m_pvtSolver.hasValidFix() || m_seedSolveFailures < SEED_TRUST_SOLVE_FAILURES; }

    std::unique_ptr<Acquisition> m_acquisition;                     ///< Acquisition engine.
    Settings::Configuration m_configuration;                        ///< Application configuration.

    std::unique_ptr<CaCodeGenerator> m_code;                        ///< C/A code generator.
    std::unique_ptr<SpectrumEngine> m_gpu;                          ///< GPU/CPU compute back-end.
    static constexpr int SEED_TRUST_SOLVE_FAILURES = 10;            ///< Pre-fix solve failures before the seed is distrusted.
    PVTSolver m_pvtSolver;                                          ///< Position solver.
    int m_seedSolveFailures{0};                                     ///< Failed pre-fix solves with a snap-trusted seed.
    NavigationDecoder m_navDecoder;                                 ///< Navigation decoder.
    NmeaGenerator m_nmeaGenerator;                                  ///< NMEA sentence generator.
    Channel m_channels[GPS_CA_SV_COUNT];                            ///< Per-satellite channels.
    std::vector<PseudorangeSample> m_lastPseudorangeSamples;        ///< Set by the last computeNavigationSolution() call.

    std::shared_ptr<Sink> m_sink{nullptr};                          ///< Telemetry sink.
    Profiler m_profiler;                                            ///< Processing time profiler.
    uint32_t m_currentBlockIndex{0};                                ///< Block index of the block being processed.

    TrackingWorkerPool m_trackingPool;                              ///< Barrier pool running trackBlock per channel.
    const ComplexFloatVector *m_currentTrackInput{nullptr};         ///< Block being tracked by the current run.
    std::vector<int> m_activeChannels;                              ///< Channel indices tracked this block.

    ComplexFloatVector m_acqInputPool;                              ///< Recycled buffer for acquisition job snapshots.
    std::thread m_acquisitionThread;                                ///< Background acquisition worker thread.
    BoundedQueue<AcquisitionJob> m_acquisitionJobQueue{1};          ///< Consumer-to-worker job handoff (one in flight).
    BoundedQueue<AcquisitionResult> m_acquisitionResultQueue{4};    ///< Worker-to-consumer result handoff.
    std::atomic<bool> m_acquisitionBusy{false};                     ///< True while a background search is in flight.
    int m_nextAcquisitionChannel{0};                                ///< Cursor for the cold-start sweep.
    bool m_coldStartSweepDone{false};                               ///< True once every channel has been searched once.

    TelemetryExporter m_telemetryExporter;                          ///< Background console and JSON writer.
};
}

#endif
