#ifndef INCLUDED_GPSOPENCL_COMMON_H
#define INCLUDED_GPSOPENCL_COMMON_H

/** @file GPSOpenClCommon.h
 *  @brief Common types and GPS L1 C/A constants.
 */

#include <complex>
#include <vector>

namespace GPSOpenCl
{
/** @brief Single-precision float vector. */
using FloatVector = std::vector<float>;

/** @brief Double-precision float vector. */
using DoubleVector = std::vector<double>;

/** @brief Single-precision complex sample vector. */
using ComplexFloatVector = std::vector<std::complex<float>>;

/** @brief Double-precision complex sample vector. */
using ComplexDoubleVector = std::vector<std::complex<double>>;

/** @brief Max default memory allocation size (bytes). */
const int DEFAULT_MAX_ALLOCATION(0x10'00'00);

/** @brief C/A code length (chips). */
const int GPS_CA_CODE_LENGTH(1023);

/** @brief Number of GPS L1 C/A satellites (PRN 1-32). */
const int GPS_CA_SV_COUNT(32);

/** @brief C/A code chipping rate (Hz). */
const float GPS_CA_CODE_FREQUENCY_HZ(1'023'000);

/** @brief C/A code period (seconds). */
const float GPS_CA_CODE_PERIOD_SEC(0.001);

/** @brief Ratio of L1 carrier frequency to C/A code chipping rate (1575.42 MHz / 1.023 MHz), exact by design.
 *   Doppler on the carrier and Doppler on the code scale by this same ratio, so dividing a carrier
 *   frequency estimate by it yields the matching code-rate correction (carrier-aided code loop). */
const float GPS_L1_CARRIER_TO_CODE_RATIO(1540.0);

/** @brief Complex imaginary unit (0 + 1j). */
const std::complex<float> IMAGINARY_UNIT(0.0f, 1.0f);

/** @brief Software name string. */
inline const std::string SOFTWARE_NAME = "GPSOpenCl";

/** @brief Software version string. */
inline const std::string SOFTWARE_VERSION = "v0.0.1";
}

#endif
