#ifndef INCLUDED_GPSOPENCL_NMEAGENERATOR_HPP
#define INCLUDED_GPSOPENCL_NMEAGENERATOR_HPP

/** @file GPSOpenClNmeaGenerator.hpp
 *  @brief NMEA-0183 sentence generator with checksum computation.
 */

#include "Pvt/GPSOpenClPVTSolver.hpp"
#include "Tracking/GPSOpenClChannel.hpp"

#include <ctime>
#include <string>
#include <vector>

#include "Common/GPSOpenClStructs.hpp"

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

    /** @brief GPS to UTC leap second offset (s), 2017-01-01 value. The receiver does not decode
     *   the UTC parameters from subframe 4 page 18, so the current constant is compiled in. */
    static constexpr int GPS_UTC_LEAP_SECONDS = 18;

    /** @brief Convert GPS time to Unix UTC time. The 10-bit broadcast week number resolves into
     *   the rollover era starting 2019-11-17 (week 2048), valid until 2039.
     *  @param gpsWeekNumber Broadcast GPS week number (10-bit).
     *  @param gpsTowSec     GPS time of week (s).
     *  @return Unix UTC time (s). */
    static time_t gpsToUnixTime(int gpsWeekNumber, double gpsTowSec);

    /** @brief Generate GGA sentence string. UTC time of day derives from GPS time including the
     *   leap second offset.
     *  @param solution      PVT solution.
     *  @param numSatellites Number of satellites used.
     *  @param gpsTowSec     GPS time of week (s).
     *  @param gpsWeekNumber Broadcast GPS week number (10-bit).
     *  @return GGA sentence. */
    static std::string generateGgga(const ReceiverPvtSolution &solution, int numSatellites, double gpsTowSec, int gpsWeekNumber);

    /** @brief Generate RMC sentence string. UTC time and date derive from GPS time including the
     *   leap second offset. No wall clock is read, so recorded or simulated data keeps a
     *   consistent time and date.
     *  @param solution      PVT solution.
     *  @param gpsTowSec     GPS time of week (s).
     *  @param gpsWeekNumber Broadcast GPS week number (10-bit).
     *  @return RMC sentence. */
    static std::string generateGprmc(const ReceiverPvtSolution &solution, double gpsTowSec, int gpsWeekNumber);

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
    static std::string generateGpgsv(const Channel channels[GPS_CA_SV_COUNT], const EcefPosition &rxEcef, bool rxPositionValid);

    /** @brief Generate individual GSV sentences.
     *  @param channels        Satellite channel array.
     *  @param rxEcef          Receiver ECEF position, used to compute real az/el per satellite.
     *  @param rxPositionValid True if rxEcef holds a valid solution.
     *  @return Vector of GSV sentences. Missing fix or ephemeris leaves azimuth and elevation
     *          empty. Values are never fabricated. */
    static std::vector<std::string> generateGpgsvSentences(const Channel channels[GPS_CA_SV_COUNT], const EcefPosition &rxEcef, bool rxPositionValid);

    /** @brief Wrap an already generated sentence into the wire output struct.
     *  @param sentence Complete NMEA sentence.
     *  @return NMEA output struct. */
    static NmeaGeneratorOutput outputFromSentence(const std::string &sentence);

    /** @brief Generate GSA as output struct.
     *  @param solution   PVT solution.
     *  @param activePrns Active satellite PRNs.
     *  @return NMEA output struct. */
    static NmeaGeneratorOutput generateGpgsaOutput(const ReceiverPvtSolution &solution, const std::vector<int> &activePrns);

    /** @brief Generate GSV as output struct vector.
     *  @param channels        Satellite channel array.
     *  @param rxEcef          Receiver ECEF position, used to compute real az/el per satellite.
     *  @param rxPositionValid True if rxEcef holds a valid solution.
     *  @return Vector of NMEA output structs. */
    static std::vector<NmeaGeneratorOutput> generateGpgsvOutput(const Channel channels[GPS_CA_SV_COUNT], const EcefPosition &rxEcef, bool rxPositionValid);

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
