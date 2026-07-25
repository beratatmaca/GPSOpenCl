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

    size_t m_navBitOffset;                ///< Current nav bit offset.
    uint8_t m_seenSubframeMask;           ///< Bitmask of decoded subframes.
    GpsEphemeris m_accumulatedEphemeris;  ///< Accumulated ephemeris data.
    double m_lastSubframeTow;             ///< Last subframe TOW (s).
    size_t m_lastSubframeStartSample;     ///< Last subframe start sample.
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_CHANNEL_H
