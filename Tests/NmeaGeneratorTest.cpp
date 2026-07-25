#include "GPSOpenClNmeaGenerator.h"

#include "gtest/gtest.h"
#include <string>

namespace GPSOpenClTest
{
TEST(NmeaGeneratorTest, ChecksumCalculation)
{
    // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
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
} // namespace GPSOpenClTest
