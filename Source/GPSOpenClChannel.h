#ifndef INCLUDED_GPSOPENCL_CHANNEL_H
#define INCLUDED_GPSOPENCL_CHANNEL_H

#include "GPSOpenClCommon.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClTracking.h"

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
    ComplexFloatVector m_promptHistory;
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_CHANNEL_H
