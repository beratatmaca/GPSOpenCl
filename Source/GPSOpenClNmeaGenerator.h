#ifndef INCLUDED_GPSOPENCL_NMEAGENERATOR_H
#define INCLUDED_GPSOPENCL_NMEAGENERATOR_H

#include "GPSOpenClChannel.h"
#include "GPSOpenClPVTSolver.h"

#include <string>
#include <vector>

namespace GPSOpenCl
{
class NmeaGenerator
{
  public:
    NmeaGenerator();
    ~NmeaGenerator();

    static std::string generateGgga(const ReceiverPvtSolution &solution, int numSatellites, double utcTimeSec);
    static std::string generateGprmc(const ReceiverPvtSolution &solution, double utcTimeSec);
    static std::string generateGpgsa(const ReceiverPvtSolution &solution, const std::vector<int> &activePrns);
    static std::string generateGpgsv(const Channel channels[GPS_CA_SV_COUNT]);

    static std::string formatLatitude(double latDegrees);
    static std::string formatLongitude(double lonDegrees);
    static uint8_t calculateChecksum(const std::string &sentenceBody);
    static std::string appendChecksum(const std::string &sentenceBody);
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_NMEAGENERATOR_H
