#ifndef INCLUDED_GPSOPENCL_CODE_H
#define INCLUDED_GPSOPENCL_CODE_H

#include <vector>

#include "GPSOpenClCommon.h"
#include "GPSOpenClGPUCompute.h"
#include "GPSOpenClSettings.h"

namespace GPSOpenCl
{
class Code
{
  public:
    Code();
    Code(Settings::Configuration conf);
    ~Code();

    char m_caCode[GPS_CA_SV_COUNT][GPS_CA_CODE_LENGTH];
    std::vector<FloatVector> m_upsampledCaCode;
    std::vector<ComplexFloatVector> m_upsampledFreqDomainCaCode;

    void setConfiguration(Settings::Configuration conf);
    void createLookupTable(Compute *gpu);

  private:
    void initialize();
    void calculateCACode();

    int m_sampleLength;
    float m_samplingFrequencyHz;
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_CODE_H