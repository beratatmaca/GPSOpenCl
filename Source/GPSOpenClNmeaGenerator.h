#ifndef INCLUDED_GPSOPENCL_NMEAGENERATOR_H
#define INCLUDED_GPSOPENCL_NMEAGENERATOR_H

/** @file GPSOpenClNmeaGenerator.h
 *  @brief NMEA-0183 sentence generator with checksum computation.
 */

#include "GPSOpenClChannel.h"
#include "GPSOpenClPVTSolver.h"

#include <string>
#include <vector>

#include "GPSOpenClStructs.h"

namespace GPSOpenCl
{
/** @brief NMEA-0183 sentence generator (GGA, RMC, GSA, GSV). */
class NmeaGenerator
{
  public:
    NmeaGenerator();

    /** @brief Construct from generator parameters.
     *  @param input NMEA enable flags. */
    NmeaGenerator(const NmeaGeneratorInput &input);

    ~NmeaGenerator() = default;
    NmeaGenerator(const NmeaGenerator &) = delete;
    NmeaGenerator &operator=(const NmeaGenerator &) = delete;
    NmeaGenerator(NmeaGenerator &&) = delete;
    NmeaGenerator &operator=(NmeaGenerator &&) = delete;

    /** @brief Check if GGA output is enabled.
     *  @return True if enabled. */
    bool isGgaEnabled() const { return m_inputConfig.enableGga != 0; }

    /** @brief Check if RMC output is enabled.
     *  @return True if enabled. */
    bool isRmcEnabled() const { return m_inputConfig.enableRmc != 0; }

    /** @brief Check if GSA output is enabled.
     *  @return True if enabled. */
    bool isGsaEnabled() const { return m_inputConfig.enableGsa != 0; }

    /** @brief Check if GSV output is enabled.
     *  @return True if enabled. */
    bool isGsvEnabled() const { return m_inputConfig.enableGsv != 0; }

    /** @brief Generate GGA sentence string.
     *  @param solution      PVT solution.
     *  @param numSatellites Number of satellites used.
     *  @param utcTimeSec    UTC time (s).
     *  @return GGA sentence. */
    static std::string generateGgga(const ReceiverPvtSolution &solution, int numSatellites, double utcTimeSec);

    /** @brief Generate RMC sentence string.
     *  @param solution   PVT solution.
     *  @param utcTimeSec UTC time (s).
     *  @return RMC sentence. */
    static std::string generateGprmc(const ReceiverPvtSolution &solution, double utcTimeSec);

    /** @brief Generate GSA sentence string.
     *  @param solution   PVT solution.
     *  @param activePrns Active satellite PRNs.
     *  @return GSA sentence. */
    static std::string generateGpgsa(const ReceiverPvtSolution &solution, const std::vector<int> &activePrns);

    /** @brief Generate GSV sentence string (all sentences concatenated).
     *  @param channels        Satellite channel array.
     *  @param rxEcef          Receiver ECEF position, used to compute real az/el per satellite.
     *  @param rxPositionValid True if rxEcef holds a valid solution.
     *  @return GSV sentences. */
    static std::string
        generateGpgsv(const Channel channels[GPS_CA_SV_COUNT], const EcefPosition &rxEcef, bool rxPositionValid);

    /** @brief Generate individual GSV sentences.
     *  @param channels        Satellite channel array.
     *  @param rxEcef          Receiver ECEF position, used to compute real az/el per satellite.
     *  @param rxPositionValid True if rxEcef holds a valid solution.
     *  @return Vector of GSV sentences. Missing fix or ephemeris leaves azimuth and elevation
     *          empty. Values are never fabricated. */
    static std::vector<std::string> generateGpgsvSentences(const Channel channels[GPS_CA_SV_COUNT],
                                                           const EcefPosition &rxEcef,
                                                           bool rxPositionValid);

    /** @brief Generate GGA as output struct.
     *  @param solution      PVT solution.
     *  @param numSatellites Number of satellites used.
     *  @param utcTimeSec    UTC time (s).
     *  @return NMEA output struct. */
    static NmeaGeneratorOutput
        generateGggaOutput(const ReceiverPvtSolution &solution, int numSatellites, double utcTimeSec);

    /** @brief Generate RMC as output struct.
     *  @param solution   PVT solution.
     *  @param utcTimeSec UTC time (s).
     *  @return NMEA output struct. */
    static NmeaGeneratorOutput generateGprmcOutput(const ReceiverPvtSolution &solution, double utcTimeSec);

    /** @brief Generate GGA as output struct from PvtSolverOutput.
     *  @param pvtOutput     PVT output struct.
     *  @param numSatellites Number of satellites used.
     *  @param utcTimeSec    UTC time (s).
     *  @return NMEA output struct. */
    static NmeaGeneratorOutput
        generateGggaOutput(const PvtSolverOutput &pvtOutput, int numSatellites, double utcTimeSec);

    /** @brief Generate RMC as output struct from PvtSolverOutput.
     *  @param pvtOutput  PVT output struct.
     *  @param utcTimeSec UTC time (s).
     *  @return NMEA output struct. */
    static NmeaGeneratorOutput generateGprmcOutput(const PvtSolverOutput &pvtOutput, double utcTimeSec);

    /** @brief Generate GSA as output struct.
     *  @param solution   PVT solution.
     *  @param activePrns Active satellite PRNs.
     *  @return NMEA output struct. */
    static NmeaGeneratorOutput generateGpgsaOutput(const ReceiverPvtSolution &solution,
                                                   const std::vector<int> &activePrns);

    /** @brief Generate GSV as output struct vector.
     *  @param channels        Satellite channel array.
     *  @param rxEcef          Receiver ECEF position, used to compute real az/el per satellite.
     *  @param rxPositionValid True if rxEcef holds a valid solution.
     *  @return Vector of NMEA output structs. */
    static std::vector<NmeaGeneratorOutput>
        generateGpgsvOutput(const Channel channels[GPS_CA_SV_COUNT], const EcefPosition &rxEcef, bool rxPositionValid);

    /** @brief Format latitude for NMEA output.
     *  @param latDegrees Latitude (deg).
     *  @return Formatted latitude string. */
    static std::string formatLatitude(double latDegrees);

    /** @brief Format longitude for NMEA output.
     *  @param lonDegrees Longitude (deg).
     *  @return Formatted longitude string. */
    static std::string formatLongitude(double lonDegrees);

    /** @brief Compute 8-bit XOR checksum.
     *  @param sentenceBody Sentence body (between $ and *).
     *  @return Checksum byte. */
    static uint8_t calculateChecksum(const std::string &sentenceBody);

    /** @brief Append checksum to sentence.
     *  @param sentenceBody Sentence body.
     *  @return Complete sentence with *hh checksum. */
    static std::string appendChecksum(const std::string &sentenceBody);

  private:
    NmeaGeneratorInput m_inputConfig;    ///< Generator enable flags.
};
}

#endif
