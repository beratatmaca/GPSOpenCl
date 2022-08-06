#ifndef INCLUDED_GPSOPENCL_ACQUISITION_H
#define INCLUDED_GPSOPENCL_ACQUISITION_H

#include <algorithm>
#include <vector>

#include "GPSOpenClChannel.h"
#include "GPSOpenClCode.h"
#include "GPSOpenClCommon.h"
#include "GPSOpenClGPUCompute.h"
#include "GPSOpenClSettings.h"

namespace GPSOpenCl
{
class Acquisition
{
  public:
    Acquisition(Settings::Configuration conf);
    ~Acquisition();

    void correlate(const ComplexFloatVector input, Compute *gpu, Code *code, Channel *acqChannel);

  private:
    void createDopplerSearchTable();
    void exp(int length, float frequency, float samplingRate, float phaseOffset, ComplexFloatVector *output);

    std::vector<ComplexFloatVector> m_dopplerSearch;
    int m_numberOfFreqencyBins;
    float m_initialFrequency;
    float m_freqSpacing;
    int m_length;
    float m_samplingFrequency;
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_ACQUISITION_H
