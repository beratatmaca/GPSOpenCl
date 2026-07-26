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

namespace GPSOpenCl
{
/** @brief Per-channel acquisition/tracking lifecycle state. */
enum class ChannelState : uint32_t
{
    Acquiring = 0,  ///< No tracking loop running; eligible for acquisition search.
    Confirming = 1, ///< Tracking loop running, lock not yet confirmed.
    Tracking = 2    ///< Confirmed lock; feeds nav decode and PVT.
};

/** @brief Per-satellite acquisition, tracking, and navigation channel. */
class Channel
{
  public:
    Channel();
    ~Channel();

    int m_svId; ///< Satellite vehicle ID.

    /** @brief Store acquisition peak metrics.
     *  @param peakValue     Correlation peak magnitude.
     *  @param peakIndex     Code phase of peak (samples).
     *  @param peakFrequency Doppler at peak (Hz).
     *  @param meanValue     Mean correlation level. */
    void insertAcquisitionMetrics(float peakValue, int peakIndex, float peakFrequency, float meanValue);

    /** @brief Retrieve acquisition results.
     *  @param peakIndex     Code phase of peak (output).
     *  @param peakValue     Peak magnitude (output).
     *  @param peakFrequency Doppler at peak (output).
     *  @param meanValue     Mean level (output).
     *  @param cno           C/N0 (output).
     *  @param peakRatio     Peak-to-mean ratio (output). */
    void getAcquisitionResults(int *peakIndex, float *peakValue, float *peakFrequency, float *meanValue, float *cno,
                               float *peakRatio);

    /** @brief Check if acquisition threshold is met. */
    void checkAcquisition();

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

    /** @brief Get accumulated prompt correlator history.
     *  @return Prompt I/Q history. */
    const ComplexFloatVector &getPromptHistory() const { return m_promptHistory; }

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
    static ChannelState computeNextState(ChannelState current, bool goodBlock, int &confirmProgress,
                                         int &lossProgress, int &blocksInConfirming, int confirmDebounceBlocks,
                                         int confirmTimeoutBlocks, int lossDebounceBlocks);

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

  private:
    /** @brief Reset navigation decode state without reinitializing the tracking loop. */
    void resetNavigationState();

    /** @brief Evaluate lock quality and drive channel state transitions. */
    void evaluateLockState();

    int m_acquisitionPeakIndex;           ///< Peak code phase (samples).
    float m_acquisitionPeakValue;         ///< Peak correlation magnitude.
    float m_acquisitionPeakFrequency;     ///< Doppler at peak (Hz).
    float m_acquisitionMeanValue;         ///< Mean correlation level.
    float m_acquisitionCN0;               ///< Carrier-to-noise ratio (dB-Hz).
    float m_acquisitionPeakRatio;         ///< Peak-to-mean ratio.
    float m_acquisitionProcessingGain;    ///< Processing gain (dB).

    bool m_isAcquired;                    ///< True if satellite acquired.
    Tracking *m_tracking;                 ///< Tracking engine instance.
    std::shared_ptr<Sink> m_sink{nullptr};///< Telemetry sink.
    ComplexFloatVector m_promptHistory;   ///< Prompt correlator history.

    int m_bitSyncPhase;                   ///< Locked sample-level bit-edge phase, 0-19 (-1 = not yet synced).
    size_t m_navBitOffset;                ///< Current nav bit offset.
    uint8_t m_seenSubframeMask;           ///< Bitmask of decoded subframes.
    GpsEphemeris m_accumulatedEphemeris;  ///< Accumulated ephemeris data.
    double m_lastSubframeTow;             ///< Last subframe TOW (s).
    size_t m_lastSubframeStartSample;     ///< Last subframe start sample.

    ChannelState m_state;                 ///< Current lifecycle state.
    int m_confirmProgress;                ///< Leaky-bucket progress toward confirming lock.
    int m_lossProgress;                   ///< Leaky-bucket progress toward declaring lock lost.
    int m_blocksInConfirming;             ///< Blocks spent in Confirming since last acquisition.
    float m_carrierLockThreshold;         ///< Min carrier lock indicator to count as locked.
    float m_codeLockRatioTolerance;       ///< Max |codeLockRatio - 1.0| to count as locked.
    int m_confirmDebounceBlocks;          ///< Good blocks needed to confirm tracking.
    int m_confirmTimeoutBlocks;           ///< Blocks before abandoning an unconfirmed acquisition.
    int m_lossDebounceBlocks;             ///< Bad blocks needed to declare lock lost.
};
}

#endif
