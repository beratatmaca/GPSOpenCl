#include "Common/GPSOpenClSettings.hpp"
#include "Tracking/GPSOpenClChannel.hpp"

#include "gtest/gtest.h"

namespace GPSOpenClTest
{
TEST(ChannelStateMachineTest, ConfirmingAdvancesToTrackingAfterDebounce)
{
    int confirmProgress = 0, lossProgress = 0, blocksInConfirming = 0;
    GPSOpenCl::ChannelState state = GPSOpenCl::ChannelState::Confirming;

    for (int i = 0; i < 49; i++)
    {
        state = GPSOpenCl::Channel::computeNextState(state, true, confirmProgress, lossProgress, blocksInConfirming, 50, 200, 100);
        EXPECT_EQ(state, GPSOpenCl::ChannelState::Confirming);
    }

    state = GPSOpenCl::Channel::computeNextState(state, true, confirmProgress, lossProgress, blocksInConfirming, 50, 200, 100);
    EXPECT_EQ(state, GPSOpenCl::ChannelState::Tracking);
}

TEST(ChannelStateMachineTest, ConfirmingLeakyBucketToleratesOccasionalBadBlock)
{
    int confirmProgress = 0, lossProgress = 0, blocksInConfirming = 0;
    GPSOpenCl::ChannelState state = GPSOpenCl::ChannelState::Confirming;

    for (int i = 0; i < 10; i++)
    {
        state = GPSOpenCl::Channel::computeNextState(state, true, confirmProgress, lossProgress, blocksInConfirming, 50, 200, 100);
    }
    EXPECT_EQ(confirmProgress, 10);

    state = GPSOpenCl::Channel::computeNextState(state, false, confirmProgress, lossProgress, blocksInConfirming, 50, 200, 100);
    EXPECT_EQ(confirmProgress, 9);
    EXPECT_EQ(state, GPSOpenCl::ChannelState::Confirming);
}

TEST(ChannelStateMachineTest, ConfirmingTimesOutBackToAcquiring)
{
    int confirmProgress = 0, lossProgress = 0, blocksInConfirming = 0;
    GPSOpenCl::ChannelState state = GPSOpenCl::ChannelState::Confirming;

    for (int i = 0; i < 199; i++)
    {
        state = GPSOpenCl::Channel::computeNextState(state, false, confirmProgress, lossProgress, blocksInConfirming, 50, 200, 100);
        EXPECT_EQ(state, GPSOpenCl::ChannelState::Confirming);
    }

    state = GPSOpenCl::Channel::computeNextState(state, false, confirmProgress, lossProgress, blocksInConfirming, 50, 200, 100);
    EXPECT_EQ(state, GPSOpenCl::ChannelState::Acquiring);
}

TEST(ChannelStateMachineTest, TrackingDropsToAcquiringAfterSustainedLoss)
{
    int confirmProgress = 0, lossProgress = 0, blocksInConfirming = 0;
    GPSOpenCl::ChannelState state = GPSOpenCl::ChannelState::Tracking;

    for (int i = 0; i < 99; i++)
    {
        state = GPSOpenCl::Channel::computeNextState(state, false, confirmProgress, lossProgress, blocksInConfirming, 50, 200, 100);
        EXPECT_EQ(state, GPSOpenCl::ChannelState::Tracking);
    }

    state = GPSOpenCl::Channel::computeNextState(state, false, confirmProgress, lossProgress, blocksInConfirming, 50, 200, 100);
    EXPECT_EQ(state, GPSOpenCl::ChannelState::Acquiring);
}

TEST(ChannelStateMachineTest, TrackingRecoversFromTransientLossWithoutDroppingState)
{
    int confirmProgress = 0, lossProgress = 0, blocksInConfirming = 0;
    GPSOpenCl::ChannelState state = GPSOpenCl::ChannelState::Tracking;

    for (int i = 0; i < 80; i++)
    {
        state = GPSOpenCl::Channel::computeNextState(state, false, confirmProgress, lossProgress, blocksInConfirming, 50, 200, 100);
    }
    EXPECT_EQ(lossProgress, 80);
    EXPECT_EQ(state, GPSOpenCl::ChannelState::Tracking);

    for (int i = 0; i < 80; i++)
    {
        state = GPSOpenCl::Channel::computeNextState(state, true, confirmProgress, lossProgress, blocksInConfirming, 50, 200, 100);
    }
    EXPECT_EQ(lossProgress, 0);
    EXPECT_EQ(state, GPSOpenCl::ChannelState::Tracking);
}

class ChannelTest : public testing::Test
{
  public:
    GPSOpenCl::Settings m_settings;
    GPSOpenCl::Channel m_channel;

  protected:
    void SetUp() override
    {
        m_settings.captureSettings();
        m_channel.svId = 1;
    }
};

TEST_F(ChannelTest, StartsInAcquiringState)
{
    EXPECT_EQ(m_channel.getState(), GPSOpenCl::ChannelState::Acquiring);
    EXPECT_TRUE(m_channel.isEligibleForAcquisition());
    EXPECT_FALSE(m_channel.isTrackingLoopActive());
    EXPECT_FALSE(m_channel.isTrackingConfirmed());
}

TEST_F(ChannelTest, InitTrackingEntersConfirmingWithEmptyPromptHistory)
{
    m_channel.setAcquired(true);
    m_channel.initTracking(m_settings.configuration, 0.0f, 0.0f);

    EXPECT_EQ(m_channel.getState(), GPSOpenCl::ChannelState::Confirming);
    EXPECT_TRUE(m_channel.isTrackingLoopActive());
    EXPECT_FALSE(m_channel.isTrackingConfirmed());
    EXPECT_TRUE(m_channel.getPromptHistory().empty());
}

TEST_F(ChannelTest, SustainedNonLockTimesOutBackToAcquiringWithoutFeedingPromptHistory)
{
    m_channel.setAcquired(true);
    m_channel.initTracking(m_settings.configuration, 0.0f, 0.0f);

    int codeLength = m_settings.configuration.acquisitionInput.numberOfSamplesPerCode;
    GPSOpenCl::ComplexFloatVector zeroInput(codeLength, std::complex<float>(0.0f, 0.0f));

    int confirmTimeoutBlocks = m_settings.configuration.trackingInput.confirmTimeoutBlocks;
    for (int i = 0; i < confirmTimeoutBlocks; i++)
    {
        m_channel.trackBlock(zeroInput);
        EXPECT_TRUE(m_channel.getPromptHistory().empty());
    }

    EXPECT_EQ(m_channel.getState(), GPSOpenCl::ChannelState::Acquiring);
    EXPECT_FALSE(m_channel.isAcquired());
}

TEST(ChannelEphemerisMergeTest, ConsistentDataSetAccumulatesToComplete)
{
    GPSOpenCl::GpsEphemeris accumulated{};
    uint8_t mask = 0;

    GPSOpenCl::GpsEphemeris sf1{};
    sf1.subframeId = 1;
    sf1.iodc = 0x123;
    sf1.af0 = 1.0e-4;
    GPSOpenCl::Channel::mergeSubframe(sf1, accumulated, mask);

    GPSOpenCl::GpsEphemeris sf2{};
    sf2.subframeId = 2;
    sf2.iode2 = 0x23;
    sf2.sqrtA = 5153.0;
    GPSOpenCl::Channel::mergeSubframe(sf2, accumulated, mask);

    GPSOpenCl::GpsEphemeris sf3{};
    sf3.subframeId = 3;
    sf3.iode3 = 0x23;
    sf3.i0 = 0.95;
    GPSOpenCl::Channel::mergeSubframe(sf3, accumulated, mask);

    EXPECT_EQ(mask, 0x7);
    EXPECT_EQ(accumulated.af0, 1.0e-4);
    EXPECT_EQ(accumulated.sqrtA, 5153.0);
    EXPECT_EQ(accumulated.i0, 0.95);
}

TEST(ChannelEphemerisMergeTest, NewOrbitDataSetInvalidatesStaleClock)
{
    GPSOpenCl::GpsEphemeris accumulated{};
    uint8_t mask = 0;

    GPSOpenCl::GpsEphemeris sf1{};
    sf1.subframeId = 1;
    sf1.iodc = 0x123;
    GPSOpenCl::Channel::mergeSubframe(sf1, accumulated, mask);
    EXPECT_EQ(mask, 0x1);

    GPSOpenCl::GpsEphemeris sf2{};
    sf2.subframeId = 2;
    sf2.iode2 = 0x24;
    GPSOpenCl::Channel::mergeSubframe(sf2, accumulated, mask);

    EXPECT_EQ(mask, 0x2);
}

TEST(ChannelEphemerisMergeTest, NewClockDataSetInvalidatesStaleOrbit)
{
    GPSOpenCl::GpsEphemeris accumulated{};
    uint8_t mask = 0;

    GPSOpenCl::GpsEphemeris sf2{};
    sf2.subframeId = 2;
    sf2.iode2 = 0x23;
    GPSOpenCl::Channel::mergeSubframe(sf2, accumulated, mask);

    GPSOpenCl::GpsEphemeris sf3{};
    sf3.subframeId = 3;
    sf3.iode3 = 0x23;
    GPSOpenCl::Channel::mergeSubframe(sf3, accumulated, mask);
    EXPECT_EQ(mask, 0x6);

    GPSOpenCl::GpsEphemeris sf1{};
    sf1.subframeId = 1;
    sf1.iodc = 0x124;
    GPSOpenCl::Channel::mergeSubframe(sf1, accumulated, mask);

    EXPECT_EQ(mask, 0x1);
}

TEST(ChannelEphemerisMergeTest, SyncOnlySubframesRefreshHeaderWithoutTouchingMask)
{
    GPSOpenCl::GpsEphemeris accumulated{};
    uint8_t mask = 0x7;

    GPSOpenCl::GpsEphemeris sf5{};
    sf5.subframeId = 5;
    sf5.tow = 12345.0;
    GPSOpenCl::Channel::mergeSubframe(sf5, accumulated, mask);

    EXPECT_EQ(mask, 0x7);
    EXPECT_EQ(accumulated.tow, 12345.0);
    EXPECT_EQ(accumulated.subframeId, 5);
}
}
