#include "GPSOpenClNmeaGenerator.h"

#include "GPSOpenClAtmosphericCorrections.h"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <sstream>

using namespace GPSOpenCl;

NmeaGenerator::NmeaGenerator()
    : m_inputConfig{STRUCT_VERSION_1, 1, 1, 1, 1}
{
}

NmeaGenerator::NmeaGenerator(const NmeaGeneratorInput &input)
    : m_inputConfig(input)
{
}

NmeaGenerator::~NmeaGenerator()
{
}

NmeaGeneratorOutput NmeaGenerator::generateGggaOutput(const ReceiverPvtSolution &solution, int numSatellites, double utcTimeSec)
{
    std::string str = generateGgga(solution, numSatellites, utcTimeSec);
    NmeaGeneratorOutput out{};
    out.structVersion = STRUCT_VERSION_1;
    snprintf(out.sentence, sizeof(out.sentence), "%s", str.c_str());
    return out;
}

NmeaGeneratorOutput NmeaGenerator::generateGprmcOutput(const ReceiverPvtSolution &solution, double utcTimeSec)
{
    std::string str = generateGprmc(solution, utcTimeSec);
    NmeaGeneratorOutput out{};
    out.structVersion = STRUCT_VERSION_1;
    snprintf(out.sentence, sizeof(out.sentence), "%s", str.c_str());
    return out;
}

NmeaGeneratorOutput NmeaGenerator::generateGggaOutput(const PvtSolverOutput &pvtOutput, int numSatellites, double utcTimeSec)
{
    ReceiverPvtSolution sol = PVTSolver::outputToSolution(pvtOutput);
    return generateGggaOutput(sol, numSatellites, utcTimeSec);
}

NmeaGeneratorOutput NmeaGenerator::generateGprmcOutput(const PvtSolverOutput &pvtOutput, double utcTimeSec)
{
    ReceiverPvtSolution sol = PVTSolver::outputToSolution(pvtOutput);
    return generateGprmcOutput(sol, utcTimeSec);
}

uint8_t NmeaGenerator::calculateChecksum(const std::string &sentenceBody)
{
    uint8_t checksum = 0;
    for (char c : sentenceBody)
    {
        checksum ^= static_cast<uint8_t>(c);
    }
    return checksum;
}

std::string NmeaGenerator::appendChecksum(const std::string &sentenceBody)
{
    uint8_t checksum = calculateChecksum(sentenceBody);
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
    char hemisphere = (latDegrees >= 0.0) ? 'N' : 'S';
    double absLat = std::fabs(latDegrees);
    int degrees = static_cast<int>(absLat);
    double minutes = (absLat - degrees) * 60.0;

    char buf[32];
    std::snprintf(buf, sizeof(buf), "%02d%07.4f,%c", degrees, minutes, hemisphere);
    return std::string(buf);
}

std::string NmeaGenerator::formatLongitude(double lonDegrees)
{
    if (!std::isfinite(lonDegrees))
    {
        return "00000.0000,E";
    }
    char hemisphere = (lonDegrees >= 0.0) ? 'E' : 'W';
    double absLon = std::fabs(lonDegrees);
    int degrees = static_cast<int>(absLon);
    double minutes = (absLon - degrees) * 60.0;

    char buf[32];
    std::snprintf(buf, sizeof(buf), "%03d%07.4f,%c", degrees, minutes, hemisphere);
    return std::string(buf);
}

std::string NmeaGenerator::generateGgga(const ReceiverPvtSolution &solution, int numSatellites, double utcTimeSec)
{
    int hours = static_cast<int>(utcTimeSec / 3600.0) % 24;
    int minutes = static_cast<int>(fmod(utcTimeSec, 3600.0) / 60.0);
    double seconds = fmod(utcTimeSec, 60.0);

    char timeBuf[16];
    std::snprintf(timeBuf, sizeof(timeBuf), "%02d%02d%05.2f", hours, minutes, seconds);

    std::string latStr = formatLatitude(solution.geodeticPosition.latitude);
    std::string lonStr = formatLongitude(solution.geodeticPosition.longitude);

    int fixQuality = solution.isValid ? 1 : 0;
    double hdop = solution.isValid ? solution.dopHDOP : 99.9;
    double alt = solution.isValid ? solution.geodeticPosition.altitude : 0.0;

    char body[256];
    std::snprintf(body, sizeof(body), "GPGGA,%s,%s,%s,%d,%02d,%.1f,%.1f,M,0.0,M,,",
                  timeBuf, latStr.c_str(), lonStr.c_str(), fixQuality, numSatellites, hdop, alt);

    return appendChecksum(body);
}

std::string NmeaGenerator::generateGprmc(const ReceiverPvtSolution &solution, double utcTimeSec)
{
    int hours = static_cast<int>(utcTimeSec / 3600.0) % 24;
    int minutes = static_cast<int>(fmod(utcTimeSec, 3600.0) / 60.0);
    double seconds = fmod(utcTimeSec, 60.0);

    char timeBuf[16];
    std::snprintf(timeBuf, sizeof(timeBuf), "%02d%02d%05.2f", hours, minutes, seconds);

    char status = solution.isValid ? 'A' : 'V';
    std::string latStr = formatLatitude(solution.geodeticPosition.latitude);
    std::string lonStr = formatLongitude(solution.geodeticPosition.longitude);

    char dateBuf[8] = "000000";
    {
        time_t now = std::time(nullptr);
        struct tm utcTm{};
        if (gmtime_r(&now, &utcTm) != nullptr)
        {
            std::snprintf(dateBuf, sizeof(dateBuf), "%02d%02d%02d",
                          utcTm.tm_mday, utcTm.tm_mon + 1, utcTm.tm_year % 100);
        }
    }

    char body[256];
    std::snprintf(body, sizeof(body), "GPRMC,%s,%c,%s,%s,0.0,0.0,%s,,,A",
                  timeBuf, status, latStr.c_str(), lonStr.c_str(), dateBuf);

    return appendChecksum(body);
}

std::string NmeaGenerator::generateGpgsa(const ReceiverPvtSolution &solution, const std::vector<int> &activePrns)
{
    int fixMode = solution.isValid ? 3 : 1;
    double pdop = solution.isValid ? solution.dopPDOP : 99.9;
    double hdop = solution.isValid ? solution.dopHDOP : 99.9;
    double vdop = solution.isValid ? solution.dopVDOP : 99.9;

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
                                                                const EcefPosition &rxEcef, bool rxPositionValid)
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
            float peakVal = 0.0f, peakFreq = 0.0f, meanVal = 0.0f, cn0 = 0.0f, peakRatio = 0.0f;
            const_cast<Channel &>(channels[i]).getAcquisitionResults(&peakIdx, &peakVal, &peakFreq, &meanVal, &cn0, &peakRatio);

            SvInfo sv{channels[i].m_svId, static_cast<int>(std::round(cn0)), false, 0.0, 0.0};

            if (rxPositionValid && channels[i].hasCompleteEphemeris())
            {
                size_t promptCount = channels[i].getPromptHistory().size();
                size_t subframeStartSample = channels[i].getLastSubframeStartSample();
                if (promptCount >= subframeStartSample)
                {
                    double subframeStartTow = channels[i].getLastSubframeTow() - 6.0;
                    double elapsedSeconds =
                        static_cast<double>(promptCount - subframeStartSample) * GPS_CA_CODE_PERIOD_SEC;
                    double transmitTime = subframeStartTow + elapsedSeconds;

                    SatelliteOrbit orbit =
                        PVTSolver::computeSatelliteOrbit(channels[i].getAccumulatedEphemeris(), transmitTime);
                    AtmosphericCorrections::computeAzimuthElevation(rxEcef, orbit.position, sv.azimuthDeg,
                                                                    sv.elevationDeg);
                    sv.hasPosition = true;
                }
            }

            acquiredSats.push_back(sv);
        }
    }

    int totalSats = static_cast<int>(acquiredSats.size());
    int numSentences = (totalSats + 3) / 4;
    if (numSentences == 0) numSentences = 1;

    std::vector<std::string> sentences;
    for (int s = 0; s < numSentences; s++)
    {
        std::ostringstream oss;
        oss << "GPGSV," << numSentences << "," << (s + 1) << "," << totalSats;

        for (int i = 0; i < 4; i++)
        {
            int idx = s * 4 + i;
            if (idx < totalSats)
            {
                char buf[32];
                if (acquiredSats[idx].hasPosition)
                {
                    int elev = static_cast<int>(std::lround(acquiredSats[idx].elevationDeg));
                    int azim = ((static_cast<int>(std::lround(acquiredSats[idx].azimuthDeg)) % 360) + 360) % 360;
                    std::snprintf(buf, sizeof(buf), ",%02d,%02d,%03d,%02d",
                                  acquiredSats[idx].prn, elev, azim, acquiredSats[idx].snr);
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

std::string NmeaGenerator::generateGpgsv(const Channel channels[GPS_CA_SV_COUNT], const EcefPosition &rxEcef,
                                         bool rxPositionValid)
{
    std::string result;
    for (const auto &sentence : generateGpgsvSentences(channels, rxEcef, rxPositionValid))
    {
        result += sentence;
    }
    return result;
}

NmeaGeneratorOutput NmeaGenerator::generateGpgsaOutput(const ReceiverPvtSolution &solution, const std::vector<int> &activePrns)
{
    std::string str = generateGpgsa(solution, activePrns);
    NmeaGeneratorOutput out{};
    out.structVersion = STRUCT_VERSION_1;
    snprintf(out.sentence, sizeof(out.sentence), "%s", str.c_str());
    return out;
}

std::vector<NmeaGeneratorOutput> NmeaGenerator::generateGpgsvOutput(const Channel channels[GPS_CA_SV_COUNT],
                                                                     const EcefPosition &rxEcef, bool rxPositionValid)
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
