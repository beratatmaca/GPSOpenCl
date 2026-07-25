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
typedef std::vector<float> FloatVector;

/** @brief Double-precision float vector. */
typedef std::vector<double> DoubleVector;

/** @brief Single-precision complex sample vector. */
typedef std::vector<std::complex<float>> ComplexFloatVector;

/** @brief Double-precision complex sample vector. */
typedef std::vector<std::complex<double>> ComplexDoubleVector;

/** @brief Max default memory allocation size (bytes). */
const int DEFAULT_MAX_ALLOCATION(0x100000);

/** @brief C/A code length (chips). */
const int GPS_CA_CODE_LENGTH(1023);

/** @brief Number of GPS L1 C/A satellites (PRN 1-32). */
const int GPS_CA_SV_COUNT(32);

/** @brief C/A code chipping rate (Hz). */
const float GPS_CA_CODE_FREQUENCY_HZ(1023000);

/** @brief C/A code period (seconds). */
const float GPS_CA_CODE_PERIOD_SEC(0.001);

/** @brief Complex imaginary unit (0 + 1j). */
const std::complex<float> IMAGINARY_UNIT(0.0f, 1.0f);

/** @brief Software name string. */
inline const std::string SOFTWARE_NAME = "GPSOpenCl";

/** @brief Software version string. */
inline const std::string SOFTWARE_VERSION = "v0.0.1";
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_COMMON_H