#include "GPSOpenClNmeaGenerator.h"

#include "GPSOpenClAtmosphericCorrections.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <sstream>

using namespace GPSOpenCl;

NmeaGenerator::NmeaGenerator() : m_inputConfig{STRUCT_VERSION_1, 1, 1, 1, 1}
{
}

NmeaGenerator::NmeaGenerator(const NmeaGeneratorInput &input) : m_inputConfig(input)
{
}

NmeaGeneratorOutput NmeaGenerator::outputFromSentence(const std::string &sentence)
{
    NmeaGeneratorOutput out{};
    out.structVersion = STRUCT_VERSION_1;
    snprintf(out.sentence, sizeof(out.sentence), "%s", sentence.c_str());
    return out;
}

uint8_t NmeaGenerator::calculateChecksum(const std::string &sentenceBody)
{
    uint8_t checksum = 0;
    for (const char c : sentenceBody)
    {
        checksum ^= static_cast<uint8_t>(c);
    }
    return checksum;
}

std::string NmeaGenerator::appendChecksum(const std::string &sentenceBody)
{
    const uint8_t checksum = calculateChecksum(sentenceBody);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "*%02X\r\n", checksum);
    return "$" + sentenceBody + buf;
}

std::string NmeaGenerator::formatLatitude(double latDegrees)
{
    if (!std::isfinite(latDegrees))
    {
        return "0000.0000,N";
    }
    const char hemisphere = (latDegrees >= 0.0) ? 'N' : 'S';
    const double absLat = std::fabs(latDegrees);
    int degrees = static_cast<int>(absLat);
    double minutes = std::round((absLat - degrees) * 60.0 * 10000.0) / 10000.0;
    if (minutes >= 60.0)
    {
        minutes -= 60.0;
        degrees += 1;
    }

    char buf[32];
    std::snprintf(buf, sizeof(buf), "%02d%07.4f,%c", degrees, minutes, hemisphere);
    return {buf};
}

std::string NmeaGenerator::formatLongitude(double lonDegrees)
{
    if (!std::isfinite(lonDegrees))
    {
        return "00000.0000,E";
    }
    const char hemisphere = (lonDegrees >= 0.0) ? 'E' : 'W';
    const double absLon = std::fabs(lonDegrees);
    int degrees = static_cast<int>(absLon);
    double minutes = std::round((absLon - degrees) * 60.0 * 10000.0) / 10000.0;
    if (minutes >= 60.0)
    {
        minutes -= 60.0;
        degrees += 1;
    }

    char buf[32];
    std::snprintf(buf, sizeof(buf), "%03d%07.4f,%c", degrees, minutes, hemisphere);
    return {buf};
}

time_t NmeaGenerator::gpsToUnixTime(int gpsWeekNumber, double gpsTowSec)
{
    const int fullWeek = (gpsWeekNumber % 1024) + 2048;
    const double gpsEpochUnixSec = 315964800.0;
    return static_cast<time_t>(
        gpsEpochUnixSec + (static_cast<double>(fullWeek) * 604800.0) + gpsTowSec - GPS_UTC_LEAP_SECONDS);
}

std::string NmeaGenerator::generateGgga(const ReceiverPvtSolution &solution,
                                        int numSatellites,
                                        double gpsTowSec,
                                        int gpsWeekNumber)
{
    const time_t utcTime = gpsToUnixTime(gpsWeekNumber, gpsTowSec);

    struct tm utcTm{};

    gmtime_r(&utcTime, &utcTm);
    const double fractional = gpsTowSec - std::floor(gpsTowSec);
    const double seconds = static_cast<double>(utcTm.tm_sec) + fractional;

    char timeBuf[16];
    std::snprintf(timeBuf, sizeof(timeBuf), "%02d%02d%05.2f", utcTm.tm_hour, utcTm.tm_min, seconds);

    const std::string latStr = formatLatitude(solution.geodeticPosition.latitudeDeg);
    const std::string lonStr = formatLongitude(solution.geodeticPosition.longitudeDeg);

    const int fixQuality = solution.isValid ? 1 : 0;
    const double hdop = solution.isValid ? solution.dopHDOP : 99.9;
    const double alt = solution.isValid ? solution.geodeticPosition.altitudeMeters : 0.0;

    char body[256];
    std::snprintf(body,
                  sizeof(body),
                  "GPGGA,%s,%s,%s,%d,%02d,%.1f,%.1f,M,0.0,M,,",
                  timeBuf,
                  latStr.c_str(),
                  lonStr.c_str(),
                  fixQuality,
                  numSatellites,
                  hdop,
                  alt);

    return appendChecksum(body);
}

std::string NmeaGenerator::generateGprmc(const ReceiverPvtSolution &solution, double gpsTowSec, int gpsWeekNumber)
{
    const time_t utcTime = gpsToUnixTime(gpsWeekNumber, gpsTowSec);

    struct tm utcTm{};

    gmtime_r(&utcTime, &utcTm);
    const double fractional = gpsTowSec - std::floor(gpsTowSec);
    const double seconds = static_cast<double>(utcTm.tm_sec) + fractional;

    char timeBuf[16];
    std::snprintf(timeBuf, sizeof(timeBuf), "%02d%02d%05.2f", utcTm.tm_hour, utcTm.tm_min, seconds);

    const char status = solution.isValid ? 'A' : 'V';
    const std::string latStr = formatLatitude(solution.geodeticPosition.latitudeDeg);
    const std::string lonStr = formatLongitude(solution.geodeticPosition.longitudeDeg);

    char dateBuf[8];
    std::snprintf(dateBuf, sizeof(dateBuf), "%02d%02d%02d", utcTm.tm_mday, utcTm.tm_mon + 1, utcTm.tm_year % 100);

    char body[256];
    std::snprintf(body,
                  sizeof(body),
                  "GPRMC,%s,%c,%s,%s,0.0,0.0,%s,,,A",
                  timeBuf,
                  status,
                  latStr.c_str(),
                  lonStr.c_str(),
                  dateBuf);

    return appendChecksum(body);
}

std::string NmeaGenerator::generateGpgsa(const ReceiverPvtSolution &solution, const std::vector<int> &activePrns)
{
    const int fixMode = solution.isValid ? 3 : 1;
    const double pdop = solution.isValid ? solution.dopPDOP : 99.9;
    const double hdop = solution.isValid ? solution.dopHDOP : 99.9;
    const double vdop = solution.isValid ? solution.dopVDOP : 99.9;

    std::ostringstream oss;
    oss << "GPGSA,A," << fixMode;

    for (size_t i = 0; i < 12; i++)
    {
        if (i < activePrns.size())
        {
            char buf[8];
            std::snprintf(buf, sizeof(buf), ",%02d", activePrns[i]);
            oss << buf;
        }
        else
        {
            oss << ",";
        }
    }

    char dopBuf[64];
    std::snprintf(dopBuf, sizeof(dopBuf), ",%.1f,%.1f,%.1f", pdop, hdop, vdop);
    oss << dopBuf;

    return appendChecksum(oss.str());
}

std::vector<std::string> NmeaGenerator::generateGpgsvSentences(const Channel channels[GPS_CA_SV_COUNT],
                                                               const EcefPosition &rxEcef,
                                                               bool rxPositionValid)
{
    struct SvInfo
    {
        int prn;
        int snr;
        bool hasPosition;
        double elevationDeg;
        double azimuthDeg;
    };

    std::vector<SvInfo> acquiredSats;

    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        if (channels[i].isAcquired())
        {
            int peakIdx = 0;
            float peakVal = 0.0f;
            float peakFreq = 0.0f;
            float meanVal = 0.0f;
            float cn0 = 0.0f;
            float peakRatio = 0.0f;
            channels[i].getAcquisitionResults(&peakIdx, &peakVal, &peakFreq, &meanVal, &cn0, &peakRatio);

            SvInfo sv{channels[i].svId, static_cast<int>(std::round(cn0)), false, 0.0, 0.0};

            if (rxPositionValid && channels[i].hasCompleteEphemeris())
            {
                const size_t promptCount = channels[i].getPromptHistory().size();
                const size_t subframeStartSample = channels[i].getLastSubframeStartSample();
                if (promptCount >= subframeStartSample)
                {
                    const double subframeStartTow = channels[i].getLastSubframeTow() - 6.0;
                    const double elapsedSeconds =
                        static_cast<double>(promptCount - subframeStartSample) * GPS_CA_CODE_PERIOD_SEC;
                    const double transmitTime = subframeStartTow + elapsedSeconds;

                    const SatelliteOrbit orbit =
                        PVTSolver::computeSatelliteOrbit(channels[i].getAccumulatedEphemeris(), transmitTime);
                    AtmosphericCorrections::computeAzimuthElevation(
                        rxEcef, orbit.position, sv.azimuthDeg, sv.elevationDeg);
                    sv.hasPosition = true;
                }
            }

            acquiredSats.push_back(sv);
        }
    }

    const int totalSats = static_cast<int>(acquiredSats.size());
    int numSentences = (totalSats + 3) / 4;
    if (numSentences == 0)
    {
        numSentences = 1;
    }

    std::vector<std::string> sentences;
    for (int s = 0; s < numSentences; s++)
    {
        std::ostringstream oss;
        oss << "GPGSV," << numSentences << "," << (s + 1) << "," << totalSats;

        for (int i = 0; i < 4; i++)
        {
            const int idx = (s * 4) + i;
            if (idx < totalSats)
            {
                char buf[32];
                if (acquiredSats[idx].hasPosition)
                {
                    const int elev = std::clamp(static_cast<int>(std::lround(acquiredSats[idx].elevationDeg)), 0, 90);
                    const int azim = ((static_cast<int>(std::lround(acquiredSats[idx].azimuthDeg)) % 360) + 360) % 360;
                    std::snprintf(buf,
                                  sizeof(buf),
                                  ",%02d,%02d,%03d,%02d",
                                  acquiredSats[idx].prn,
                                  elev,
                                  azim,
                                  acquiredSats[idx].snr);
                }
                else
                {
                    std::snprintf(buf, sizeof(buf), ",%02d,,,%02d", acquiredSats[idx].prn, acquiredSats[idx].snr);
                }
                oss << buf;
            }
            else if (totalSats > 0)
            {
                oss << ",,,,";
            }
        }

        sentences.push_back(appendChecksum(oss.str()));
    }

    return sentences;
}

std::string NmeaGenerator::generateGpgsv(const Channel channels[GPS_CA_SV_COUNT],
                                         const EcefPosition &rxEcef,
                                         bool rxPositionValid)
{
    std::string result;
    for (const auto &sentence : generateGpgsvSentences(channels, rxEcef, rxPositionValid))
    {
        result += sentence;
    }
    return result;
}

NmeaGeneratorOutput NmeaGenerator::generateGpgsaOutput(const ReceiverPvtSolution &solution,
                                                       const std::vector<int> &activePrns)
{
    const std::string str = generateGpgsa(solution, activePrns);
    NmeaGeneratorOutput out{};
    out.structVersion = STRUCT_VERSION_1;
    snprintf(out.sentence, sizeof(out.sentence), "%s", str.c_str());
    return out;
}

std::vector<NmeaGeneratorOutput> NmeaGenerator::generateGpgsvOutput(const Channel channels[GPS_CA_SV_COUNT],
                                                                    const EcefPosition &rxEcef,
                                                                    bool rxPositionValid)
{
    std::vector<NmeaGeneratorOutput> outputs;
    for (const auto &sentence : generateGpgsvSentences(channels, rxEcef, rxPositionValid))
    {
        NmeaGeneratorOutput out{};
        out.structVersion = STRUCT_VERSION_1;
        snprintf(out.sentence, sizeof(out.sentence), "%s", sentence.c_str());
        outputs.push_back(out);
    }
    return outputs;
}
