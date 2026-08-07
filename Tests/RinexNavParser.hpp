#ifndef INCLUDED_GPSOPENCLTEST_RINEXNAVPARSER_HPP
#define INCLUDED_GPSOPENCLTEST_RINEXNAVPARSER_HPP

#include "NavDecode/GPSOpenClNavigationDecoder.hpp"

#include <cmath>
#include <ctime>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace GPSOpenClTest
{
/** @brief Minimal RINEX 2 navigation message reader, used only to obtain independent ephemeris ground truth. */
class RinexNavParser
{
  public:
    static bool findEphemeris(const std::string &path, int prn, GPSOpenCl::GpsEphemeris *ephem)
    {
        std::ifstream file(path);
        if (!file.is_open())
        {
            return false;
        }

        std::string line;
        while (std::getline(file, line))
        {
            if (line.find("END OF HEADER") != std::string::npos)
            {
                break;
            }
        }

        while (std::getline(file, line))
        {
            if (line.size() < 2)
            {
                continue;
            }

            int recordPrn = 0;
            int year = 0, month = 0, day = 0, hour = 0, minute = 0;
            double sec = 0.0;
            {
                std::istringstream headerStream(line);
                headerStream >> recordPrn >> year >> month >> day >> hour >> minute >> sec;
            }

            std::vector<std::string> recordLines;
            recordLines.push_back(line);
            bool ok = true;
            for (int i = 1; i < 8; i++)
            {
                std::string next;
                if (!std::getline(file, next))
                {
                    ok = false;
                    break;
                }
                recordLines.push_back(next);
            }
            if (!ok)
            {
                continue;
            }

            if (recordPrn != prn)
            {
                continue;
            }

            std::vector<double> values = extractDFields(recordLines);
            if (values.size() < 31)
            {
                continue;
            }

            ephem->svId = recordPrn;
            ephem->af0 = values[0];
            ephem->af1 = values[1];
            ephem->af2 = values[2];
            ephem->Crs = values[4];
            ephem->deltaN = values[5];
            ephem->M0 = values[6];
            ephem->Cuc = values[7];
            ephem->e = values[8];
            ephem->Cus = values[9];
            ephem->sqrtA = values[10];
            ephem->toe = values[11];
            ephem->Cic = values[12];
            ephem->omega0 = values[13];
            ephem->Cis = values[14];
            ephem->i0 = values[15];
            ephem->Crc = values[16];
            ephem->omega = values[17];
            ephem->omegaDot = values[18];
            ephem->idot = values[19];
            ephem->weekNumber = static_cast<int>(std::lround(values[21]));
            ephem->tgd = values[25];

            double towSec = 0.0;
            if (!epochToGpsSecondsOfWeek(year, month, day, hour, minute, sec, ephem->weekNumber, &towSec))
            {
                return false;
            }
            ephem->toc = towSec;
            ephem->tow = towSec;
            ephem->subframeId = 0;
            ephem->isValid = true;

            return true;
        }

        return false;
    }

  private:
    static std::vector<double> extractDFields(const std::vector<std::string> &recordLines)
    {
        static const std::regex dFieldPattern(R"(-?\d\.\d+D[+-]\d+)");
        std::vector<double> values;

        for (const std::string &line : recordLines)
        {
            auto begin = std::sregex_iterator(line.begin(), line.end(), dFieldPattern);
            auto end = std::sregex_iterator();
            for (auto it = begin; it != end; ++it)
            {
                std::string token = it->str();
                for (char &c : token)
                {
                    if (c == 'D' || c == 'd')
                    {
                        c = 'E';
                    }
                }
                values.push_back(std::stod(token));
            }
        }

        return values;
    }

    static bool epochToGpsSecondsOfWeek(int year, int month, int day, int hour, int minute, double sec, int gpsWeek, double *outTowSec)
    {
        int fullYear = (year < 80) ? (2000 + year) : (1900 + year);

        std::tm tmEpoch{};
        tmEpoch.tm_year = fullYear - 1900;
        tmEpoch.tm_mon = month - 1;
        tmEpoch.tm_mday = day;
        tmEpoch.tm_hour = hour;
        tmEpoch.tm_min = minute;
        tmEpoch.tm_sec = 0;

        time_t epochUnix = timegm(&tmEpoch);
        if (epochUnix == static_cast<time_t>(-1))
        {
            return false;
        }

        const time_t gpsEpochUnix = 315'964'800;    // 1980-01-06T00:00:00Z
        double secondsSinceGpsEpoch = static_cast<double>(epochUnix - gpsEpochUnix) + sec;

        double secondsSinceWeekStart = secondsSinceGpsEpoch - static_cast<double>(gpsWeek) * 604800.0;
        *outTowSec = secondsSinceWeekStart;
        return true;
    }
};
}    // namespace GPSOpenClTest

#endif    //! INCLUDED_GPSOPENCLTEST_RINEXNAVPARSER_HPP
