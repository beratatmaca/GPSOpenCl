#include "GPSOpenClApplication.h"

#include "gtest/gtest.h"
#include <cmath>
#include <limits>
#include <string>

namespace GPSOpenClTest
{
TEST(TelemetryJsonTest, NonFiniteValuesEmitValidJson)
{
    GPSOpenCl::Application::TelemetrySnapshot snapshot{};
    snapshot.utcTimeSec = std::numeric_limits<double>::quiet_NaN();
    snapshot.solution.geodeticPosition.latitudeDeg = std::numeric_limits<double>::infinity();
    snapshot.solution.geodeticPosition.longitudeDeg = -std::numeric_limits<double>::infinity();
    snapshot.solution.dopHDOP = std::numeric_limits<double>::quiet_NaN();

    GPSOpenCl::Application::SatelliteTelemetry sat{};
    sat.prn = 7;
    sat.cn0 = std::numeric_limits<float>::quiet_NaN();
    snapshot.satellites.push_back(sat);

    const std::string jsonText = GPSOpenCl::Application::formatTelemetryJson(snapshot);

    EXPECT_EQ(jsonText.find("nan"), std::string::npos);
    EXPECT_EQ(jsonText.find("inf"), std::string::npos);
    EXPECT_NE(jsonText.find("\"latitude\": 0"), std::string::npos);
    EXPECT_NE(jsonText.find("\"cn0\": 0"), std::string::npos);
}

TEST(TelemetryJsonTest, FiniteValuesPassThrough)
{
    GPSOpenCl::Application::TelemetrySnapshot snapshot{};
    snapshot.solution.isValid = true;
    snapshot.solution.geodeticPosition.latitudeDeg = 48.1173;
    snapshot.solution.geodeticPosition.altitudeMeters = 545.4;

    const std::string jsonText = GPSOpenCl::Application::formatTelemetryJson(snapshot);

    EXPECT_NE(jsonText.find("\"valid\": true"), std::string::npos);
    EXPECT_NE(jsonText.find("48.1173"), std::string::npos);
    EXPECT_NE(jsonText.find("545.4"), std::string::npos);
}
}
