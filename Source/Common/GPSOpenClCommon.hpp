#ifndef INCLUDED_GPSOPENCL_COMMON_HPP
#define INCLUDED_GPSOPENCL_COMMON_HPP

/** @file GPSOpenClCommon.hpp
 *  @brief Common types and GPS L1 C/A constants.
 */

#include <complex>
#include <string>
#include <vector>

namespace GPSOpenCl
{
/** @brief Single-precision float vector. */
using FloatVector = std::vector<float>;

/** @brief Single-precision complex sample vector. */
using ComplexFloatVector = std::vector<std::complex<float>>;

/** @brief C/A code length (chips). */
const int GPS_CA_CODE_LENGTH = 1023;

/** @brief Number of GPS L1 C/A satellites (PRN 1-32). */
const int GPS_CA_SV_COUNT = 32;

/** @brief C/A code chipping rate (Hz). */
const float GPS_CA_CODE_FREQUENCY_HZ = 1'023'000;

/** @brief C/A code period (s). Double precision on purpose. Float 0.001 carries 5e-11 relative
 *   error. That grows to centimeter pseudorange error per second. */
constexpr double GPS_CA_CODE_PERIOD_SEC = 0.001;

/** @brief Ratio of L1 carrier to code chipping rate. Equals 1575.42 MHz over 1.023 MHz, exact.
 *   Carrier and code Doppler scale by this ratio. Dividing carrier Doppler gives the code rate
 *   correction. */
const float GPS_L1_CARRIER_TO_CODE_RATIO = 1540.0;

/** @brief Speed of light (m/s), per IS-GPS-200. */
constexpr double SPEED_OF_LIGHT_M_S = 299792458.0;

/** @brief Navigation message bits per subframe. */
constexpr int GPS_NAV_BITS_PER_SUBFRAME = 300;

/** @brief Code periods (1 ms prompt samples) per navigation data bit (50 bps). */
constexpr int GPS_NAV_CODE_PERIODS_PER_BIT = 20;

/** @brief Subframe duration (s). */
constexpr double GPS_NAV_SUBFRAME_DURATION_SEC = 6.0;

/** @brief WGS-84 semi-major axis (m). */
constexpr double WGS84_SEMI_MAJOR_AXIS_M = 6378137.0;

/** @brief WGS-84 earth rotation rate (rad/s), per IS-GPS-200. */
constexpr double WGS84_EARTH_ROTATION_RATE_RAD_S = 7.2921151467e-5;

/** @brief WGS-84 earth gravitational parameter (m^3/s^2), per IS-GPS-200. */
constexpr double WGS84_GRAVITATIONAL_PARAMETER = 3.986005e14;

/** @brief Complex imaginary unit (0 + 1j). */
const std::complex<float> IMAGINARY_UNIT = {0.0f, 1.0f};

/** @brief Software name string. */
inline const std::string SOFTWARE_NAME = "GPSOpenCl";

/** @brief Software version string. */
inline const std::string SOFTWARE_VERSION = "v0.0.1";
}

#endif
