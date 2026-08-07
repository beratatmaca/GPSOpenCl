#ifndef INCLUDED_GPSOPENCL_CHANNEL_H
#define INCLUDED_GPSOPENCL_CHANNEL_H

/** @file GPSOpenClChannel.h
 *  @brief Per-satellite processing channel.
 */

#include "GPSOpenClCommon.h"
#include "GPSOpenClNavigationDecoder.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClSink.h"
#include "GPSOpenClTracking.h"

#include <memory>
#include <mutex>
#include <string>

namespace GPSOpenCl
{
/** @brief Per-channel acquisition/tracking lifecycle state. */
enum class ChannelState : uint8_t
{
    Acquiring = 0,     ///< No tracking loop running, eligible for acquisition.
    Confirming = 1,    ///< Tracking loop running, lock not yet confirmed.
    Tracking = 2       ///< Confirmed lock, feeds nav decode and PVT.
};

/** @brief Per-satellite acquisition, tracking, and navigation channel. */
class Channel
{
  public:
    Channel();
    ~Channel();
    Channel(const Channel &) = delete;
    Channel &operator=(const Channel &) = delete;
    Channel(Channel &&) = delete;
    Channel &operator=(Channel &&) = delete;

    int svId{0};    ///< Satellite vehicle ID.

    /** @brief Store acquisition peak metrics.
     *  @param peakValue     Correlation peak magnitude.
     *  @param peakIndex     Code phase of peak (samples).
     *  @param peakFrequencyHz Doppler at peak (Hz).
     *  @param meanValue     Mean correlation level. */
    void insertAcquisitionMetrics(float peakValue, int peakIndex, float peakFrequencyHz, float meanValue);

    /** @brief Reset accumulated acquisition metrics before a fresh search sweep. */
    void resetAcquisitionMetrics();

    /** @brief Retrieve acquisition results.
     *  @param peakIndex     Code phase of peak (output).
     *  @param peakValue     Peak magnitude (output).
     *  @param peakFrequencyHz Doppler at peak (output).
     *  @param meanValue     Mean level (output).
     *  @param cnoDbHz           C/N0 (output).
     *  @param peakRatio     Peak-to-mean ratio (output). */
    void getAcquisitionResults(int *peakIndex,
                               float *peakValue,
                               float *peakFrequencyHz,
                               float *meanValue,
                               float *cnoDbHz,
                               float *peakRatio) const;

    /** @brief Enable or disable correlator timing samples. Applied when the tracking engine is
     *   created. A disabled profiler costs no clock reads.
     *  @param enabled True to take timing samples. */
    void setTrackingTimingEnabled(bool enabled) { m_trackingTimingEnabled = enabled; }

    /** @brief Get the current tracking output snapshot. Published from the consumer thread after
     *   the barrier. Never from inside the tracking workers.
     *  @param out Output struct, filled on success.
     *  @return True if a tracking engine exists. */
    bool getTrackingOutput(TrackingOutput *out) const;

    /** @brief Check if satellite is acquired.
     *  @return True if acquired. */
    bool isAcquired() const;

    /** @brief Set acquisition state.
     *  @param acquired Acquisition flag. */
    void setAcquired(bool acquired);

    /** @brief Initialize tracking from acquisition results.
     *  @param conf           Application configuration.
     *  @param dopplerHz      Acquired Doppler (Hz).
     *  @param codePhaseChips Acquired code phase (chips). */
    void initTracking(const Settings::Configuration &conf, float dopplerHz, float codePhaseChips);

    /** @brief Track one block of samples.
     *  @param input IQ samples. */
    void trackBlock(const ComplexFloatVector &input);

    /** @brief Get accumulated prompt correlator history. Call history accessors from the consumer
     *   thread only. The histories carry no locking. Workers append inside the trackSatellites()
     *   barrier. The consumer thread compacts them in updateNavigation().
     *  @return Prompt I/Q history. */
    const ComplexFloatVector &getPromptHistory() const { return m_promptHistory; }

    /** @brief Get the fused correlator duration of the last tracked block.
     *  @return Correlator duration (ms), zero if not currently tracking. */
    float getTrackingCorrelatorTimeMs() const;

    /** @brief Set telemetry sink.
     *  @param sink Sink implementation. */
    void setSink(std::shared_ptr<Sink> sink);

    /** @brief Get current channel state.
     *  @return Channel state. */
    ChannelState getState() const { return m_state; }

    /** @brief Check if tracking is confirmed and PVT-eligible.
     *  @return True if state is Tracking. */
    bool isTrackingConfirmed() const { return m_state == ChannelState::Tracking; }

    /** @brief Check if the tracking loop is running.
     *  @return True if state is Confirming or Tracking. */
    bool isTrackingLoopActive() const
    {
        return m_state == ChannelState::Confirming || m_state == ChannelState::Tracking;
    }

    /** @brief Check if channel is eligible for a new acquisition attempt.
     *  @return True if state is Acquiring. */
    bool isEligibleForAcquisition() const { return m_state == ChannelState::Acquiring; }

    /** @brief Compute the next lifecycle state from current state and lock quality (pure, no side effects).
     *  @param current               Current channel state.
     *  @param goodBlock             True if this block satisfies carrier and code lock criteria.
     *  @param confirmProgress       Confirm leaky-bucket progress (in/out).
     *  @param lossProgress          Loss leaky-bucket progress (in/out).
     *  @param blocksInConfirming    Blocks spent in Confirming since last acquisition (in/out).
     *  @param confirmDebounceBlocks Good-block progress needed to confirm tracking.
     *  @param confirmTimeoutBlocks  Blocks before abandoning an unconfirmed acquisition.
     *  @param lossDebounceBlocks    Bad-block progress needed to declare lock lost.
     *  @return Next channel state. */
    static ChannelState computeNextState(ChannelState current,
                                         bool goodBlock,
                                         int &confirmProgress,
                                         int &lossProgress,
                                         int &blocksInConfirming,
                                         int confirmDebounceBlocks,
                                         int confirmTimeoutBlocks,
                                         int lossDebounceBlocks);

    /** @brief Decode next subframe and merge into ephemeris.
     *  @param decoder Navigation decoder instance.
     *  @return True if all subframes 1-3 decoded. */
    bool updateNavigation(NavigationDecoder &decoder);

    /** @brief Check if subframes 1-3 are all decoded.
     *  @return True if complete. */
    bool hasCompleteEphemeris() const;

    /** @brief Merge one decoded subframe into accumulated ephemeris. Subframes 1-3 copy payload
     *   and set their mask bit. Subframes 4-5 only refresh the header. Issue of data must match
     *   across accumulated subframes. Mismatched subframes get their mask bit cleared. See
     *   IS-GPS-200 20.3.3.4.1, IODC low bits vs IODE. A satellite mid-upload cannot mix data sets.
     *  @param decoded          Freshly decoded subframe.
     *  @param accumulated      Accumulated ephemeris, updated in place.
     *  @param seenSubframeMask Bitmask of accumulated subframes (bit N-1 = subframe N), updated. */
    static void mergeSubframe(const GpsEphemeris &decoded, GpsEphemeris &accumulated, uint8_t &seenSubframeMask);

    /** @brief Take and clear buffered state transition messages. Transitions are detected on a
     *   worker thread. A blocking stdout write would stall the barrier. So messages wait here for
     *   the consumer thread.
     *  @return Buffered transition messages, empty if none. */
    std::string takePendingStateMessage()
    {
        std::string message = std::move(m_pendingStateMessage);
        m_pendingStateMessage.clear();
        return message;
    }

    /** @brief Get accumulated ephemeris.
     *  @return Ephemeris struct. */
    const GpsEphemeris &getAccumulatedEphemeris() const { return m_accumulatedEphemeris; }

    /** @brief Get TOW of last decoded subframe.
     *  @return Time of week (s). */
    double getLastSubframeTow() const { return m_lastSubframeTow; }

    /** @brief Get sample index of last decoded subframe.
     *  @return Sample index. */
    size_t getLastSubframeStartSample() const { return m_lastSubframeStartSample; }

    /** @brief Get the DLL code phase at a prompt sample. Gives sub-millisecond transmit time
     *   precision. Indices align one to one with getPromptHistory().
     *  @param sampleIndex Index into the code phase history.
     *  @return Code phase (chips, 0-1023), or 0 if out of range. */
    float getCodePhaseAtSample(size_t sampleIndex) const
    {
        return (sampleIndex < m_codePhaseHistory.size()) ? m_codePhaseHistory[sampleIndex] : 0.0f;
    }

    /** @brief Get unwrapped cumulative DLL drift at a sample. Relative to tracking start. Built
     *   from a running sum of per-block deltas. Each delta is far too small to wrap. The wrap
     *   window is +-511.5 chips. So differencing two samples never aliases. Indices align one to
     *   one with getPromptHistory().
     *  @param sampleIndex Index into the drift history.
     *  @return Cumulative drift (chips), or 0 if out of range. */
    float getCumulativeDriftChipsAtSample(size_t sampleIndex) const
    {
        return (sampleIndex < m_cumulativeDriftChipsHistory.size()) ? m_cumulativeDriftChipsHistory[sampleIndex] : 0.0f;
    }

    /** @brief Get the locked sample-level bit-edge phase used to align subframe search.
     *  @return Phase, 0-19 (-1 = not yet synced). */
    int getBitSyncPhase() const { return m_bitSyncPhase; }

  private:
    static constexpr size_t NAV_HISTORY_COMPACT_THRESHOLD = 30'000;    ///< History length that triggers compaction.
    static constexpr size_t NAV_HISTORY_COMPACT_MIN_DROP = 10'000;     ///< Minimum samples to drop per compaction.

    /** @brief Reset navigation decode state without reinitializing the tracking loop. */
    void resetNavigationState();

    /** @brief Discard history the decoders no longer need. Rebases sample indexed fields by the
     *   dropped count. Drops whole nav bits only, 20 samples each. The bit sync phase stays valid.
     *   History stays inside its initial reservation. The tracking path never reallocates it. */
    void compactNavigationHistory();

    /** @brief Evaluate lock quality and drive channel state transitions. */
    void evaluateLockState();

    mutable std::mutex m_acquisitionMetricsMutex;        ///< Guards the acquisition metric fields below
                                                         ///< against worker/consumer races.
    int m_acquisitionPeakIndex{0};                       ///< Peak code phase (samples).
    float m_acquisitionPeakValue{0.0f};                  ///< Peak correlation magnitude.
    float m_acquisitionPeakFrequencyHz{0.0f};            ///< Doppler at peak (Hz).
    float m_acquisitionMeanValue{0.0f};                  ///< Mean correlation level.
    float m_acquisitionCN0{0.0f};                        ///< Carrier-to-noise ratio (dB-Hz).
    float m_acquisitionPeakRatio{0.0f};                  ///< Peak-to-mean ratio.
    float m_acquisitionProcessingGain;                   ///< Processing gain (dB).

    bool m_isAcquired{false};                            ///< True if satellite acquired.
    std::unique_ptr<Tracking> m_tracking;                ///< Tracking engine instance.
    std::shared_ptr<Sink> m_sink{nullptr};               ///< Telemetry sink.
    ComplexFloatVector m_promptHistory;                  ///< Prompt correlator history.
    std::vector<float> m_codePhaseHistory;               ///< DLL residual code phase per prompt sample, aligned with
                                                         ///< m_promptHistory.
    std::vector<float> m_cumulativeDriftChipsHistory;    ///< Unwrapped running-sum drift per prompt sample,
                                                         ///< aligned with m_promptHistory.
    float m_lastRawCodePhaseForDrift{0.0f};              ///< Previous block raw code phase, for the drift delta.
    float m_cumulativeDriftChips{0.0f};                  ///< Running sum backing m_cumulativeDriftChipsHistory.

    int m_bitSyncPhase{-1};    ///< Locked sample-level bit-edge phase, 0-19 (-1 = not yet synced).
    std::vector<size_t> m_bitSyncSearchPositions;     ///< Per-candidate-phase search cursor while unsynced.
    size_t m_navBitOffset{0};                         ///< Current nav bit offset.
    uint8_t m_seenSubframeMask{0};                    ///< Bitmask of decoded subframes.
    GpsEphemeris m_accumulatedEphemeris;              ///< Accumulated ephemeris data.
    double m_lastSubframeTow{0.0};                    ///< Last subframe TOW (s).
    size_t m_lastSubframeStartSample{0};              ///< Last subframe start sample.

    ChannelState m_state{ChannelState::Acquiring};    ///< Current lifecycle state.
    std::string m_pendingStateMessage;                ///< Buffered state-transition console output.
    int m_confirmProgress{0};                         ///< Leaky-bucket progress toward confirming lock.
    int m_lossProgress{0};                            ///< Leaky-bucket progress toward declaring lock lost.
    int m_blocksInConfirming{0};                      ///< Blocks spent in Confirming since last acquisition.
    bool m_trackingTimingEnabled{true};               ///< Whether the tracking engine takes timing samples.
    float m_carrierLockThreshold{0.3f};               ///< Min carrier lock indicator to count as locked.
    float m_codeLockRatioTolerance{0.3f};             ///< Max |codeLockRatio - 1.0| to count as locked.
    int m_confirmDebounceBlocks{50};                  ///< Good blocks needed to confirm tracking.
    int m_confirmTimeoutBlocks{200};                  ///< Blocks before abandoning an unconfirmed acquisition.
    int m_lossDebounceBlocks{200};                    ///< Bad blocks needed to declare lock lost.
};
}

#endif
