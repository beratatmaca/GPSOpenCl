#include "GPSOpenClNmeaGenerator.h"

#include "gtest/gtest.h"
#include <cstring>
#include <string>

namespace GPSOpenClTest
{
TEST(NmeaGeneratorTest, ChecksumCalculation)
{
    std::string sentence = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
    uint8_t checksum = GPSOpenCl::NmeaGenerator::calculateChecksum(sentence);
    EXPECT_EQ(checksum, 0x47);
}

TEST(NmeaGeneratorTest, LatitudeFormat)
{
    std::string latNorth = GPSOpenCl::NmeaGenerator::formatLatitude(48.1173);
    EXPECT_EQ(latNorth, "4807.0380,N");

    std::string latSouth = GPSOpenCl::NmeaGenerator::formatLatitude(-33.8688);
    EXPECT_EQ(latSouth, "3352.1280,S");
}

TEST(NmeaGeneratorTest, LongitudeFormat)
{
    std::string lonEast = GPSOpenCl::NmeaGenerator::formatLongitude(11.5167);
    EXPECT_EQ(lonEast, "01131.0020,E");

    std::string lonWest = GPSOpenCl::NmeaGenerator::formatLongitude(-118.2437);
    EXPECT_EQ(lonWest, "11814.6220,W");
}

TEST(NmeaGeneratorTest, GenerateGgga)
{
    GPSOpenCl::ReceiverPvtSolution sol;
    sol.isValid = true;
    sol.geodeticPosition.latitude = 48.1173;
    sol.geodeticPosition.longitude = 11.5167;
    sol.geodeticPosition.altitude = 545.4;
    sol.dopHDOP = 0.9;

    std::string ggga = GPSOpenCl::NmeaGenerator::generateGgga(sol, 8, 45319.0);

    EXPECT_EQ(ggga.substr(0, 7), "$GPGGA,");
    EXPECT_TRUE(ggga.find(",4807.0380,N,01131.0020,E,1,08,0.9,545.4,M,0.0,M,,*") != std::string::npos);
    EXPECT_EQ(ggga.substr(ggga.length() - 2), "\r\n");
}

TEST(NmeaGeneratorTest, GenerateGprmc)
{
    GPSOpenCl::ReceiverPvtSolution sol;
    sol.isValid = true;
    sol.geodeticPosition.latitude = 48.1173;
    sol.geodeticPosition.longitude = 11.5167;

    std::string gprmc = GPSOpenCl::NmeaGenerator::generateGprmc(sol, 45319.0);

    EXPECT_EQ(gprmc.substr(0, 7), "$GPRMC,");
    EXPECT_TRUE(gprmc.find(",A,4807.0380,N,01131.0020,E,0.0,0.0,250726,,,A*") != std::string::npos);
}

TEST(NmeaGeneratorTest, GenerateGpgsaOutputMatchesString)
{
    GPSOpenCl::ReceiverPvtSolution sol;
    sol.isValid = true;
    sol.dopPDOP = 1.5;
    sol.dopHDOP = 1.0;
    sol.dopVDOP = 1.1;
    std::vector<int> activePrns = {1, 2, 3, 4};

    std::string gpgsaStr = GPSOpenCl::NmeaGenerator::generateGpgsa(sol, activePrns);
    GPSOpenCl::NmeaGeneratorOutput out = GPSOpenCl::NmeaGenerator::generateGpgsaOutput(sol, activePrns);

    EXPECT_EQ(out.structVersion, GPSOpenCl::STRUCT_VERSION_1);
    EXPECT_EQ(std::string(out.sentence), gpgsaStr);
}

TEST(NmeaGeneratorTest, GenerateGpgsvOutputSplitsIntoMultipleMessages)
{
    GPSOpenCl::Channel channels[GPSOpenCl::GPS_CA_SV_COUNT];
    for (int i = 0; i < 5; i++)
    {
        channels[i].m_svId = i + 1;
        channels[i].setAcquired(true);
        channels[i].insertAcquisitionMetrics(10.0f, 0, 1000.0f, 1.0f);
    }

    std::string fullString = GPSOpenCl::NmeaGenerator::generateGpgsv(channels);
    std::vector<GPSOpenCl::NmeaGeneratorOutput> outputs = GPSOpenCl::NmeaGenerator::generateGpgsvOutput(channels);

    EXPECT_EQ(outputs.size(), 2u);

    std::string reassembled;
    for (const auto &out : outputs)
    {
        EXPECT_EQ(out.structVersion, GPSOpenCl::STRUCT_VERSION_1);
        std::string sentence(out.sentence);
        EXPECT_EQ(sentence.substr(0, 7), "$GPGSV,");
        EXPECT_EQ(sentence.substr(sentence.length() - 2), "\r\n");
        EXPECT_LT(std::strlen(out.sentence), sizeof(out.sentence));
        reassembled += sentence;
    }
    EXPECT_EQ(reassembled, fullString);
}
}
