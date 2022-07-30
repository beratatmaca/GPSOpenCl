#ifndef INCLUDED_GPSOPENCL_COMMON_H
#define INCLUDED_GPSOPENCL_COMMON_H

#include <complex>
#include <vector>

namespace GPSOpenCl
{
typedef std::vector<float> FloatVector;
typedef std::vector<double> DoubleVector;
typedef std::vector<std::complex<float>> ComplexFloatVector;
typedef std::vector<std::complex<double>> ComplexDoubleVector;
const int DEFAULT_MAX_ALLOCATION(0x100000);
const int GPS_CA_CODE_LENGTH(1023);
const int GPS_CA_SV_COUNT(32);
const float GPS_CA_CODE_FREQUENCY_HZ(1023000);
const float GPS_CA_CODE_PERIOD_SEC(0.001);
const std::complex<float> IMAGINARY_UNIT(0.0f, 1.0f);
const std::string SOFTWARE_NAME = "GPSOpenCl";
const std::string SOFTWARE_VERSION = "v0.0.1";
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_COMMON_H