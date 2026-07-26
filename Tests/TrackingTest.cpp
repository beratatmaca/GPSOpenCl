#include "GPSOpenClTracking.h"

#include "gtest/gtest.h"

namespace GPSOpenClTest
{
TEST(TrackingLoopFilterTest, DefaultBandwidthsMatchReferenceDesign)
{
    EXPECT_NEAR(GPSOpenCl::Tracking::loopFilterTau1(25.0), 0.0004494f, 1e-6f);
    EXPECT_NEAR(GPSOpenCl::Tracking::loopFilterTau2(25.0), 0.02998f, 5e-5f);
    EXPECT_NEAR(GPSOpenCl::Tracking::loopFilterTau1(2.0), 0.07022f, 1e-4f);
    EXPECT_NEAR(GPSOpenCl::Tracking::loopFilterTau2(2.0), 0.37476f, 5e-4f);
}

TEST(TrackingLoopFilterTest, WiderBandwidthYieldsSmallerTimeConstants)
{
    EXPECT_LT(GPSOpenCl::Tracking::loopFilterTau1(50.0), GPSOpenCl::Tracking::loopFilterTau1(25.0));
    EXPECT_LT(GPSOpenCl::Tracking::loopFilterTau2(50.0), GPSOpenCl::Tracking::loopFilterTau2(25.0));
}
}
