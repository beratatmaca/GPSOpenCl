#ifndef INCLUDED_GPSOPENCL_CODE_H
#define INCLUDED_GPSOPENCL_CODE_H

/** @file GPSOpenClCode.h
 *  @brief C/A Gold code generator and lookup table.
 */

#include <vector>

#include "GPSOpenClCommon.h"
#include "GPSOpenClGPUCompute.h"
#include "GPSOpenClSettings.h"

namespace GPSOpenCl
{
/** @brief GPS L1 C/A code generator and frequency-domain lookup table. */
class Code
{
  public:
    Code();

    /** @brief Construct with configuration.
     *  @param conf Application configuration. */
    Code(Settings::Configuration conf);

    ~Code();

    char m_caCode[GPS_CA_SV_COUNT][GPS_CA_CODE_LENGTH];           ///< Raw C/A code chips per PRN.
    std::vector<FloatVector> m_upsampledCaCode;                   ///< Upsampled time-domain C/A codes.
    std::vector<ComplexFloatVector> m_upsampledFreqDomainCaCode;   ///< Frequency-domain C/A codes.

    /** @brief Set configuration.
     *  @param conf Application configuration. */
    void setConfiguration(Settings::Configuration conf);

    /** @brief Create upsampled and frequency-domain code lookup table.
     *  @param gpu Compute back-end for FFT. */
    void createLookupTable(Compute *gpu);

  private:
    /** @brief Initialize code generator. */
    void initialize();

    /** @brief Calculate C/A Gold codes for all PRNs. */
    void calculateCACode();

    int m_sampleLength;            ///< Samples per code period.
    float m_samplingFrequencyHz;   ///< Sampling rate (Hz).
};
}

#endif