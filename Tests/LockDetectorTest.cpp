#include "GPSOpenClLockDetector.h"

#include "gtest/gtest.h"

#include <cmath>

namespace GPSOpenClTest
{
TEST(LockDetectorTest, CarrierLockIndicatorIsHighWhenIpDominant)
{
    double indicator = GPSOpenCl::LockDetector::carrierLockIndicator(10.0, 0.1);
    EXPECT_NEAR(indicator, 1.0, 0.05);
}

TEST(LockDetectorTest, CarrierLockIndicatorIsLowWhenQpDominant)
{
    double indicator = GPSOpenCl::LockDetector::carrierLockIndicator(0.1, 10.0);
    EXPECT_NEAR(indicator, -1.0, 0.05);
}

TEST(LockDetectorTest, CarrierLockIndicatorIsAmbiguityTolerant)
{
    double positive = GPSOpenCl::LockDetector::carrierLockIndicator(10.0, 0.0);
    double negative = GPSOpenCl::LockDetector::carrierLockIndicator(-10.0, 0.0);
    EXPECT_NEAR(positive, negative, 1e-9);
    EXPECT_NEAR(positive, 1.0, 1e-9);
}

TEST(LockDetectorTest, CarrierLockIndicatorIsZeroWhenBothZero)
{
    EXPECT_NEAR(GPSOpenCl::LockDetector::carrierLockIndicator(0.0, 0.0), 0.0, 1e-9);
}

TEST(LockDetectorTest, CodeLockRatioIsNearOneWhenAligned)
{
    double ratio = GPSOpenCl::LockDetector::codeLockRatio(10.0, 10.0, 10.0);
    EXPECT_NEAR(ratio, 1.0, 1e-9);
}

TEST(LockDetectorTest, CodeLockRatioDeviatesWhenMisaligned)
{
    double ratio = GPSOpenCl::LockDetector::codeLockRatio(9.0, 10.0, 1.0);
    EXPECT_NEAR(ratio, 0.5, 1e-9);
    EXPECT_GT(std::fabs(ratio - 1.0), 0.3);
}

TEST(LockDetectorTest, CodeLockRatioIsZeroWhenPromptIsZero)
{
    EXPECT_NEAR(GPSOpenCl::LockDetector::codeLockRatio(5.0, 0.0, 5.0), 0.0, 1e-9);
}
}
