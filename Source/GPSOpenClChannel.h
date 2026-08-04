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

namespace GPSOpenCl
{
/** @brief Per-channel acquisition/tracking lifecycle state. */
enum class ChannelState : uint8_t
{
    Acquiring = 0,     ///< No tracking loop running; eligible for acquisition search.
    Confirming = 1,    ///< Tracking loop running, lock not yet confirmed.
    Tracking = 2       ///< Confirmed lock; feeds nav decode and PVT.
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

    int m_svId;    ///< Satellite vehicle ID.

    /** @brief Store acquisition peak metrics.
     *  @param peakValue     Correlation peak magnitude.
     *  @param peakIndex     Code phase of peak (samples).
     *  @param peakFrequency Doppler at peak (Hz).
     *  @param meanValue     Mean correlation level. */
    void insertAcquisitionMetrics(float peakValue, int peakIndex, float peakFrequency, float meanValue);

    /** @brief Reset accumulated acquisition metrics before a fresh search sweep. */
    void resetAcquisitionMetrics();

    /** @brief Retrieve acquisition results.
     *  @param peakIndex     Code phase of peak (output).
     *  @param peakValue     Peak magnitude (output).
     *  @param peakFrequency Doppler at peak (output).
     *  @param meanValue     Mean level (output).
     *  @param cno           C/N0 (output).
     *  @param peakRatio     Peak-to-mean ratio (output). */
    void getAcquisitionResults(int *peakIndex,
                               float *peakValue,
                               float *peakFrequency,
                               float *meanValue,
                               float *cno,
                               float *peakRatio) const;

    /** @brief Enable or disable tracking correlator timing samples; applied to the tracking engine
     *   when it is (re)created, so a disabled profiler costs no clock reads per block.
     *  @param enabled True to take timing samples. */
    void setTrackingTimingEnabled(bool enabled) { m_trackingTimingEnabled = enabled; }

    /** @brief Get the tracking engine's current output snapshot, for publishing from the consumer
     *   thread after the tracking barrier instead of inside the tracking workers.
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

    /** @brief Get accumulated prompt correlator history. This and every other history accessor
     *   (getCodePhaseAtSample, getCumulativeDriftChipsAtSample, getLastSubframeStartSample) must be
     *   called from the consumer thread only: the histories carry no locking, tracking workers
     *   append to them inside the trackSatellites() barrier, and updateNavigation() compacts them
     *   on the consumer thread, rebasing every sample index by the dropped count.
     *  @return Prompt I/Q history. */
    const ComplexFloatVector &getPromptHistory() const { return m_promptHistory; }

    /** @brief Get this channel's most recent block's tracking sub-stage timings (ms), or zero if not
     *   currently tracking.
     *  @param earlyLatePromptGenMs Output: earlyLatePromptGen duration (ms).
     *  @param numericOscillatorMs  Output: numericOscillator duration (ms).
     *  @param accumulatorMs        Output: accumulator duration (ms). */
    void
        getTrackingSubStageTimings(float *earlyLatePromptGenMs, float *numericOscillatorMs, float *accumulatorMs) const;

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

    /** @brief Get accumulated ephemeris.
     *  @return Ephemeris struct. */
    const GpsEphemeris &getAccumulatedEphemeris() const { return m_accumulatedEphemeris; }

    /** @brief Get TOW of last decoded subframe.
     *  @return Time of week (s). */
    double getLastSubframeTow() const { return m_lastSubframeTow; }

    /** @brief Get sample index of last decoded subframe.
     *  @return Sample index. */
    size_t getLastSubframeStartSample() const { return m_lastSubframeStartSample; }

    /** @brief Get the DLL residual code phase recorded alongside a given prompt history sample, for
     *   sub-millisecond transmit-time precision. Indices align 1:1 with getPromptHistory().
     *  @param sampleIndex Index into the prompt/code-phase history.
     *  @return Code phase (chips, 0-1023), or 0 if out of range. */
    float getCodePhaseAtSample(size_t sampleIndex) const
    {
        return (sampleIndex < m_codePhaseHistory.size()) ? m_codePhaseHistory[sampleIndex] : 0.0f;
    }

    /** @brief Get the unwrapped, cumulative sub-chip DLL drift accumulated up to a given prompt
     *   history sample, relative to tracking start. Unlike a raw code-phase difference between two
     *   samples, this cannot alias: it is built from a running sum of per-block deltas that are each
     *   individually far too small (bounded by one block's worth of Doppler-driven code-rate change)
     *   to wrap past the +-511.5 chip disambiguation window, so subtracting two samples recovers the
     *   true accumulated drift over any interval, no matter how long. Indices align 1:1 with
     *   getPromptHistory().
     *  @param sampleIndex Index into the prompt/code-phase history.
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

    /** @brief Discard prompt/code-phase/drift history older than the navigation decoder and PVT
     *   solver can still reference, rebasing every sample-indexed field by the dropped count. Only
     *   whole nav bits (multiples of 20 samples) are dropped so the bit-sync phase stays valid.
     *   Bounds per-channel memory and keeps the history vectors inside their initial reservation,
     *   so the tracking path never reallocates them. */
    void compactNavigationHistory();

    /** @brief Evaluate lock quality and drive channel state transitions. */
    void evaluateLockState();

    mutable std::mutex m_acquisitionMetricsMutex;    ///< Guards the acquisition metric fields below
                                                     ///< against concurrent access from the background
                                                     ///< acquisition worker thread and the consumer
                                                     ///< thread's telemetry export.
    int m_acquisitionPeakIndex;                      ///< Peak code phase (samples).
    float m_acquisitionPeakValue;                    ///< Peak correlation magnitude.
    float m_acquisitionPeakFrequency;                ///< Doppler at peak (Hz).
    float m_acquisitionMeanValue;                    ///< Mean correlation level.
    float m_acquisitionCN0;                          ///< Carrier-to-noise ratio (dB-Hz).
    float m_acquisitionPeakRatio;                    ///< Peak-to-mean ratio.
    float m_acquisitionProcessingGain;               ///< Processing gain (dB).

    bool m_isAcquired;                               ///< True if satellite acquired.
    std::unique_ptr<Tracking> m_tracking;            ///< Tracking engine instance.
    std::shared_ptr<Sink> m_sink{nullptr};           ///< Telemetry sink.
    ComplexFloatVector m_promptHistory;              ///< Prompt correlator history.
    std::vector<float> m_codePhaseHistory;    ///< DLL residual code phase per prompt sample, 1:1 with m_promptHistory.
    std::vector<float> m_cumulativeDriftChipsHistory;    ///< Unwrapped running-sum drift per prompt sample,
                                                         ///< 1:1 with m_promptHistory; see
                                                         ///< getCumulativeDriftChipsAtSample().
    float m_lastRawCodePhaseForDrift;                    ///< Previous block's raw (wrapped) code phase, for the
                                                         ///< per-block delta feeding m_cumulativeDriftChipsHistory.
    float m_cumulativeDriftChips;                        ///< Running sum backing m_cumulativeDriftChipsHistory.

    int m_bitSyncPhase;    ///< Locked sample-level bit-edge phase, 0-19 (-1 = not yet synced).
    std::vector<size_t> m_bitSyncSearchPositions;    ///< Per-candidate-phase search cursor while unsynced.
    size_t m_navBitOffset;                           ///< Current nav bit offset.
    uint8_t m_seenSubframeMask;                      ///< Bitmask of decoded subframes.
    GpsEphemeris m_accumulatedEphemeris;             ///< Accumulated ephemeris data.
    double m_lastSubframeTow;                        ///< Last subframe TOW (s).
    size_t m_lastSubframeStartSample;                ///< Last subframe start sample.

    ChannelState m_state;                            ///< Current lifecycle state.
    int m_confirmProgress;                           ///< Leaky-bucket progress toward confirming lock.
    int m_lossProgress;                              ///< Leaky-bucket progress toward declaring lock lost.
    int m_blocksInConfirming;                        ///< Blocks spent in Confirming since last acquisition.
    bool m_trackingTimingEnabled{true};              ///< Whether the tracking engine takes timing samples.
    float m_carrierLockThreshold;                    ///< Min carrier lock indicator to count as locked.
    float m_codeLockRatioTolerance;                  ///< Max |codeLockRatio - 1.0| to count as locked.
    int m_confirmDebounceBlocks;                     ///< Good blocks needed to confirm tracking.
    int m_confirmTimeoutBlocks;                      ///< Blocks before abandoning an unconfirmed acquisition.
    int m_lossDebounceBlocks;                        ///< Bad blocks needed to declare lock lost.
};
}

#endif
