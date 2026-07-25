#ifndef INCLUDED_GPSOPENCL_CHANNEL_H
#define INCLUDED_GPSOPENCL_CHANNEL_H

#include "GPSOpenClCommon.h"
#include "GPSOpenClNavigationDecoder.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClSink.h"
#include "GPSOpenClTracking.h"

#include <memory>

namespace GPSOpenCl
{
class Channel
{
  public:
    Channel();
    ~Channel();

    int m_svId;

    void insertAcquisitionMetrics(float peakValue, int peakIndex, float peakFrequency, float meanValue);
    void getAcquisitionResults(int *peakIndex, float *peakValue, float *peakFrequency, float *meanValue, float *cno,
                               float *peakRatio);
    void checkAcquisition();

    bool isAcquired() const;
    void setAcquired(bool acquired);
    void initTracking(const Settings::Configuration &conf, float dopplerHz, float codePhaseChips);
    void trackBlock(const ComplexFloatVector &input);
    const ComplexFloatVector &getPromptHistory() const { return m_promptHistory; }

    void setSink(std::shared_ptr<Sink> sink);

    // Attempts to decode the next available subframe from this channel's tracked prompt history and
    // merges it into the accumulated ephemeris. Returns true once subframes 1, 2 and 3 have all been
    // seen at least once, at which point getAccumulatedEphemeris()/getLastSubframeTow()/
    // getLastSubframeStartSample() describe a complete, real ephemeris (never fabricated).
    bool updateNavigation(NavigationDecoder &decoder);
    bool hasCompleteEphemeris() const;
    const GpsEphemeris &getAccumulatedEphemeris() const { return m_accumulatedEphemeris; }
    double getLastSubframeTow() const { return m_lastSubframeTow; }
    size_t getLastSubframeStartSample() const { return m_lastSubframeStartSample; }

  private:
    int m_acquisitionPeakIndex;
    float m_acquisitionPeakValue;
    float m_acquisitionPeakFrequency;
    float m_acquisitionMeanValue;
    float m_acquisitionCN0;
    float m_acquisitionPeakRatio;
    float m_acquisitionProcessingGain;

    bool m_isAcquired;
    Tracking *m_tracking;
    std::shared_ptr<Sink> m_sink{nullptr};
    ComplexFloatVector m_promptHistory;

    size_t m_navBitOffset;
    uint8_t m_seenSubframeMask;
    GpsEphemeris m_accumulatedEphemeris;
    double m_lastSubframeTow;
    size_t m_lastSubframeStartSample;
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_CHANNEL_H
