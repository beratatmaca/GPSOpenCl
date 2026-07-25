#ifndef INCLUDED_GPSOPENCL_NMEAGENERATOR_H
#define INCLUDED_GPSOPENCL_NMEAGENERATOR_H

#include "GPSOpenClChannel.h"
#include "GPSOpenClPVTSolver.h"

#include <string>
#include <vector>

#include "GPSOpenClStructs.h"

namespace GPSOpenCl
{
class NmeaGenerator
{
  public:
    NmeaGenerator();
    NmeaGenerator(const NmeaGeneratorInput &input);
    ~NmeaGenerator();

    bool isGgaEnabled() const { return m_inputConfig.enableGga != 0; }
    bool isRmcEnabled() const { return m_inputConfig.enableRmc != 0; }
    bool isGsaEnabled() const { return m_inputConfig.enableGsa != 0; }
    bool isGsvEnabled() const { return m_inputConfig.enableGsv != 0; }

    static std::string generateGgga(const ReceiverPvtSolution &solution, int numSatellites, double utcTimeSec);
    static std::string generateGprmc(const ReceiverPvtSolution &solution, double utcTimeSec);
    static std::string generateGpgsa(const ReceiverPvtSolution &solution, const std::vector<int> &activePrns);
    static std::string generateGpgsv(const Channel channels[GPS_CA_SV_COUNT]);

    // One complete "$...*hh\r\n" sentence per element (a full sky can need multiple GSV
    // sentences; generateGpgsv() above concatenates these for console/log dumping, but a single
    // NmeaGeneratorOutput.sentence[256] cannot hold all of them for a large satellite count).
    static std::vector<std::string> generateGpgsvSentences(const Channel channels[GPS_CA_SV_COUNT]);

    static NmeaGeneratorOutput generateGggaOutput(const ReceiverPvtSolution &solution, int numSatellites, double utcTimeSec);
    static NmeaGeneratorOutput generateGprmcOutput(const ReceiverPvtSolution &solution, double utcTimeSec);
    static NmeaGeneratorOutput generateGggaOutput(const PvtSolverOutput &pvtOutput, int numSatellites, double utcTimeSec);
    static NmeaGeneratorOutput generateGprmcOutput(const PvtSolverOutput &pvtOutput, double utcTimeSec);
    static NmeaGeneratorOutput generateGpgsaOutput(const ReceiverPvtSolution &solution, const std::vector<int> &activePrns);
    static std::vector<NmeaGeneratorOutput> generateGpgsvOutput(const Channel channels[GPS_CA_SV_COUNT]);

    static std::string formatLatitude(double latDegrees);
    static std::string formatLongitude(double lonDegrees);
    static uint8_t calculateChecksum(const std::string &sentenceBody);
    static std::string appendChecksum(const std::string &sentenceBody);

  private:
    NmeaGeneratorInput m_inputConfig;
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_NMEAGENERATOR_H
