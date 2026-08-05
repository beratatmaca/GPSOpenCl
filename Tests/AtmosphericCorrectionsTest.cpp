#include "GPSOpenClAtmosphericCorrections.h"

#include "gtest/gtest.h"
#include <cmath>

namespace GPSOpenClTest
{
TEST(AtmosphericCorrectionsTest, AzimuthElevationZenith)
{
    GPSOpenCl::EcefPosition rxEcef;
    rxEcef.x = 6378137.0;
    rxEcef.y = 0.0;
    rxEcef.z = 0.0;

    GPSOpenCl::EcefPosition satEcef;
    satEcef.x = 26560000.0;
    satEcef.y = 0.0;
    satEcef.z = 0.0;

    double az = 0.0, el = 0.0;
    GPSOpenCl::AtmosphericCorrections::computeAzimuthElevation(rxEcef, satEcef, az, el);

    EXPECT_NEAR(el, 90.0, 1e-3);
}

TEST(AtmosphericCorrectionsTest, SaastamoinenSeaLevelZenith)
{
    double delayMeters = GPSOpenCl::AtmosphericCorrections::saastamoinenTroposphericDelay(0.0, 90.0);

    EXPECT_NEAR(delayMeters, 2.3, 0.2);
}

TEST(AtmosphericCorrectionsTest, KlobucharDelayRange)
{
    GPSOpenCl::GeodeticPosition rxPos;
    rxPos.latitudeDeg = 40.0;
    rxPos.longitudeDeg = 30.0;
    rxPos.altitudeMeters = 100.0;

    GPSOpenCl::KlobucharParams params;
    params.alpha[0] = 0.1118e-07;
    params.alpha[1] = 0.7451e-08;
    params.alpha[2] = -0.5960e-07;
    params.alpha[3] = -0.1192e-06;

    params.beta[0] = 0.1167e+06;
    params.beta[1] = -0.2294e+05;
    params.beta[2] = -0.1311e+06;
    params.beta[3] = 0.1049e+07;

    double delayMeters =
        GPSOpenCl::AtmosphericCorrections::klobucharIonosphericDelay(rxPos, 45.0, 120.0, 45000.0, params);

    EXPECT_GT(delayMeters, 1.0);
    EXPECT_LT(delayMeters, 30.0);
}
}
