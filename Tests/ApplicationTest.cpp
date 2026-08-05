#include "GPSOpenClApplication.h"
#include "GPSOpenClCommon.h"

#include "gtest/gtest.h"
#include <vector>

namespace GPSOpenClTest
{
TEST(ApplicationTest, TransmitTimeAtBlockBoundaryHasNoSubMsTerm)
{
    const double tow = 345600.0;
    const double elapsed = 2.4;
    const double t = GPSOpenCl::Application::computeTransmitTime(tow, elapsed, 0.0, 1023.0);
    EXPECT_NEAR(t, tow + elapsed, 1e-12);
}

TEST(ApplicationTest, TransmitTimeMidBlockAnchorSubtractsHalfCodePeriod)
{
    const double tow = 345600.0;
    const double t = GPSOpenCl::Application::computeTransmitTime(tow, 0.0, 0.0, 511.5);
    EXPECT_NEAR(t, tow - 0.5e-3, 1e-12);
}

TEST(ApplicationTest, TransmitTimeDriftChipsAddLinearly)
{
    const double tow = 345600.0;
    const double t = GPSOpenCl::Application::computeTransmitTime(tow, 0.0, 1.023, 1023.0);
    EXPECT_NEAR(t, tow + 1.0e-6, 1e-15);
}

TEST(ApplicationTest, SnapCorrectsSingleWholeMsOutlier)
{
    const double epoch = 100.0;
    std::vector<double> transmitTimes = {1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> impliedArrivals = {epoch, epoch, epoch, epoch, epoch + 1.0e-3 + 2.0e-5};

    GPSOpenCl::Application::snapTransmitTimesToMedianArrival(transmitTimes, impliedArrivals);

    EXPECT_NEAR(transmitTimes[0], 1.0, 1e-12);
    EXPECT_NEAR(transmitTimes[1], 2.0, 1e-12);
    EXPECT_NEAR(transmitTimes[2], 3.0, 1e-12);
    EXPECT_NEAR(transmitTimes[3], 4.0, 1e-12);
    EXPECT_NEAR(transmitTimes[4], 5.0 - 1.0e-3, 1e-12);
}

TEST(ApplicationTest, SnapLeavesSubMillisecondOffsetsAlone)
{
    const double epoch = 100.0;
    std::vector<double> transmitTimes = {1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> impliedArrivals = {epoch, epoch, epoch, epoch, epoch + 0.4e-3};

    GPSOpenCl::Application::snapTransmitTimesToMedianArrival(transmitTimes, impliedArrivals);

    for (size_t i = 0; i < transmitTimes.size(); i++)
    {
        EXPECT_NEAR(transmitTimes[i], static_cast<double>(i + 1), 1e-12);
    }
}

TEST(ApplicationTest, SnapGuardBandRejectsAmbiguousOffsets)
{
    const double epoch = 100.0;
    std::vector<double> transmitTimes = {1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> impliedArrivals = {epoch, epoch, epoch, epoch, epoch + 1.3e-3};

    GPSOpenCl::Application::snapTransmitTimesToMedianArrival(transmitTimes, impliedArrivals);

    for (size_t i = 0; i < transmitTimes.size(); i++)
    {
        EXPECT_NEAR(transmitTimes[i], static_cast<double>(i + 1), 1e-12);
    }
}

TEST(ApplicationTest, SnapAlignsEvenSplitOntoOneCommonCluster)
{
    const double epoch = 100.0;
    std::vector<double> transmitTimes = {1.0, 2.0, 3.0, 4.0};
    std::vector<double> impliedArrivals = {epoch, epoch, epoch + 1.0e-3, epoch + 1.0e-3};

    GPSOpenCl::Application::snapTransmitTimesToMedianArrival(transmitTimes, impliedArrivals);

    std::vector<double> snappedArrivals(impliedArrivals.size());
    for (size_t i = 0; i < impliedArrivals.size(); i++)
    {
        snappedArrivals[i] = impliedArrivals[i] + (transmitTimes[i] - static_cast<double>(i + 1));
    }
    for (size_t i = 1; i < snappedArrivals.size(); i++)
    {
        EXPECT_NEAR(snappedArrivals[i], snappedArrivals[0], 0.5e-3);
    }
}

TEST(ApplicationTest, SnapCorrectsMinorityAgainstOddMedian)
{
    const double epoch = 100.0;
    std::vector<double> transmitTimes = {1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> impliedArrivals = {epoch, epoch, epoch, epoch - 2.0e-3, epoch + 1.0e-3};

    GPSOpenCl::Application::snapTransmitTimesToMedianArrival(transmitTimes, impliedArrivals);

    EXPECT_NEAR(transmitTimes[3], 4.0 + 2.0e-3, 1e-12);
    EXPECT_NEAR(transmitTimes[4], 5.0 - 1.0e-3, 1e-12);
    EXPECT_NEAR(transmitTimes[0], 1.0, 1e-12);
}

TEST(ApplicationTest, SnapHandlesEmptyAndMismatchedInputs)
{
    std::vector<double> transmitTimes;
    std::vector<double> impliedArrivals;
    GPSOpenCl::Application::snapTransmitTimesToMedianArrival(transmitTimes, impliedArrivals);
    EXPECT_TRUE(transmitTimes.empty());

    transmitTimes = {1.0, 2.0};
    impliedArrivals = {100.0};
    GPSOpenCl::Application::snapTransmitTimesToMedianArrival(transmitTimes, impliedArrivals);
    EXPECT_NEAR(transmitTimes[0], 1.0, 1e-12);
    EXPECT_NEAR(transmitTimes[1], 2.0, 1e-12);
}
}
