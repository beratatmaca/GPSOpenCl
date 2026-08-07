#ifndef INCLUDED_GPSOPENCL_CACODEGENERATOR_H
#define INCLUDED_GPSOPENCL_CACODEGENERATOR_H

/** @file GPSOpenClCaCodeGenerator.h
 *  @brief C/A Gold code generator and lookup table.
 */

#include <array>
#include <vector>

#include "GPSOpenClCommon.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClSpectrumEngine.h"

namespace GPSOpenCl
{
/** @brief GPS L1 C/A code generator and frequency-domain lookup table. */
class CaCodeGenerator
{
  public:
    CaCodeGenerator();

    /** @brief Construct with configuration.
     *  @param conf Application configuration. */
    CaCodeGenerator(const Settings::Configuration &conf);

    ~CaCodeGenerator();
    CaCodeGenerator(const CaCodeGenerator &) = delete;
    CaCodeGenerator &operator=(const CaCodeGenerator &) = delete;
    CaCodeGenerator(CaCodeGenerator &&) = delete;
    CaCodeGenerator &operator=(CaCodeGenerator &&) = delete;

    char caCode[GPS_CA_SV_COUNT][GPS_CA_CODE_LENGTH]{};           ///< Raw C/A code chips per PRN.
    std::vector<FloatVector> upsampledCaCode;                     ///< Upsampled time-domain C/A codes.
    std::vector<ComplexFloatVector> upsampledFreqDomainCaCode;    ///< Frequency-domain C/A codes.

    /** @brief Shared raw C/A chip table. The Gold codes depend on nothing configurable, so all
     *   consumers read one process-wide table instead of regenerating 32 KB per instance.
     *  @return Chips (+1/-1) for all PRNs, indexed [svIndex][chip]. */
    static const std::array<std::array<char, GPS_CA_CODE_LENGTH>, GPS_CA_SV_COUNT> &rawCaCodes();

    /** @brief Set configuration.
     *  @param conf Application configuration. */
    void setConfiguration(const Settings::Configuration &conf);

    /** @brief Create upsampled and frequency-domain code lookup table.
     *  @param gpu Compute back-end for FFT. */
    void createLookupTable(SpectrumEngine *gpu);

  private:
    /** @brief Initialize code generator. */
    void initialize();

    /** @brief Calculate C/A Gold codes for all PRNs. */
    void calculateCACode();

    int m_sampleLength{};             ///< Samples per code period.
    float m_samplingFrequencyHz{};    ///< Sampling rate (Hz).
};
}

#endif
