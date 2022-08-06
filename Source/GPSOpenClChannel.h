#ifndef INCLUDED_GPSOPENCL_CHANNEL_H
#define INCLUDED_GPSOPENCL_CHANNEL_H

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

  private:
    int m_acquisitionPeakIndex;
    float m_acquisitionPeakValue;
    float m_acquisitionPeakFrequency;
    float m_acquisitionMeanValue;
    float m_acquisitionCN0;
    float m_acquisitionPeakRatio;
    float m_acquisitionProcessingGain;
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_CHANNEL_H
