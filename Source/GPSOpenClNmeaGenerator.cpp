#include "GPSOpenClNmeaGenerator.h"

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <sstream>

using namespace GPSOpenCl;

NmeaGenerator::NmeaGenerator()
{
}

NmeaGenerator::~NmeaGenerator()
{
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

    char body[256];
    std::snprintf(body, sizeof(body), "GPRMC,%s,%c,%s,%s,0.0,0.0,250726,,,A",
                  timeBuf, status, latStr.c_str(), lonStr.c_str());

    return appendChecksum(body);
}

std::string NmeaGenerator::generateGpgsa(const ReceiverPvtSolution &solution, const std::vector<int> &activePrns)
{
    int fixMode = solution.isValid ? 3 : 1; // 3 = 3D fix, 1 = No fix
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

std::string NmeaGenerator::generateGpgsv(const Channel channels[GPS_CA_SV_COUNT])
{
    struct SvInfo
    {
        int prn;
        int snr;
    };
    std::vector<SvInfo> acquiredSats;

    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        if (channels[i].isAcquired())
        {
            int peakIdx = 0;
            float peakVal = 0.0f, peakFreq = 0.0f, meanVal = 0.0f, cn0 = 0.0f, peakRatio = 0.0f;
            const_cast<Channel &>(channels[i]).getAcquisitionResults(&peakIdx, &peakVal, &peakFreq, &meanVal, &cn0, &peakRatio);
            acquiredSats.push_back({channels[i].m_svId, static_cast<int>(std::round(cn0))});
        }
    }

    int totalSats = static_cast<int>(acquiredSats.size());
    int numSentences = (totalSats + 3) / 4;
    if (numSentences == 0) numSentences = 1;

    std::string result;
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
                // Nominal elevation 45 deg, azimuth 120 deg, SNR cn0
                std::snprintf(buf, sizeof(buf), ",%02d,45,120,%02d", acquiredSats[idx].prn, acquiredSats[idx].snr);
                oss << buf;
            }
            else if (totalSats > 0)
            {
                oss << ",,,,";
            }
        }

        result += appendChecksum(oss.str());
    }

    return result;
}
