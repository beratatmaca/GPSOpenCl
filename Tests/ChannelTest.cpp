#include "GPSOpenClChannel.h"
#include "GPSOpenClSettings.h"

#include "gtest/gtest.h"

namespace GPSOpenClTest
{
TEST(ChannelStateMachineTest, ConfirmingAdvancesToTrackingAfterDebounce)
{
    int confirmProgress = 0, lossProgress = 0, blocksInConfirming = 0;
    GPSOpenCl::ChannelState state = GPSOpenCl::ChannelState::Confirming;

    for (int i = 0; i < 49; i++)
    {
        state = GPSOpenCl::Channel::computeNextState(state, true, confirmProgress, lossProgress, blocksInConfirming,
                                                      50, 200, 100);
        EXPECT_EQ(state, GPSOpenCl::ChannelState::Confirming);
    }

    state = GPSOpenCl::Channel::computeNextState(state, true, confirmProgress, lossProgress, blocksInConfirming,
                                                 50, 200, 100);
    EXPECT_EQ(state, GPSOpenCl::ChannelState::Tracking);
}

TEST(ChannelStateMachineTest, ConfirmingLeakyBucketToleratesOccasionalBadBlock)
{
    int confirmProgress = 0, lossProgress = 0, blocksInConfirming = 0;
    GPSOpenCl::ChannelState state = GPSOpenCl::ChannelState::Confirming;

    for (int i = 0; i < 10; i++)
    {
        state = GPSOpenCl::Channel::computeNextState(state, true, confirmProgress, lossProgress, blocksInConfirming,
                                                      50, 200, 100);
    }
    EXPECT_EQ(confirmProgress, 10);

    state = GPSOpenCl::Channel::computeNextState(state, false, confirmProgress, lossProgress, blocksInConfirming,
                                                 50, 200, 100);
    EXPECT_EQ(confirmProgress, 9);
    EXPECT_EQ(state, GPSOpenCl::ChannelState::Confirming);
}

TEST(ChannelStateMachineTest, ConfirmingTimesOutBackToAcquiring)
{
    int confirmProgress = 0, lossProgress = 0, blocksInConfirming = 0;
    GPSOpenCl::ChannelState state = GPSOpenCl::ChannelState::Confirming;

    for (int i = 0; i < 199; i++)
    {
        state = GPSOpenCl::Channel::computeNextState(state, false, confirmProgress, lossProgress, blocksInConfirming,
                                                      50, 200, 100);
        EXPECT_EQ(state, GPSOpenCl::ChannelState::Confirming);
    }

    state = GPSOpenCl::Channel::computeNextState(state, false, confirmProgress, lossProgress, blocksInConfirming,
                                                 50, 200, 100);
    EXPECT_EQ(state, GPSOpenCl::ChannelState::Acquiring);
}

TEST(ChannelStateMachineTest, TrackingDropsToAcquiringAfterSustainedLoss)
{
    int confirmProgress = 0, lossProgress = 0, blocksInConfirming = 0;
    GPSOpenCl::ChannelState state = GPSOpenCl::ChannelState::Tracking;

    for (int i = 0; i < 99; i++)
    {
        state = GPSOpenCl::Channel::computeNextState(state, false, confirmProgress, lossProgress, blocksInConfirming,
                                                      50, 200, 100);
        EXPECT_EQ(state, GPSOpenCl::ChannelState::Tracking);
    }

    state = GPSOpenCl::Channel::computeNextState(state, false, confirmProgress, lossProgress, blocksInConfirming,
                                                 50, 200, 100);
    EXPECT_EQ(state, GPSOpenCl::ChannelState::Acquiring);
}

TEST(ChannelStateMachineTest, TrackingRecoversFromTransientLossWithoutDroppingState)
{
    int confirmProgress = 0, lossProgress = 0, blocksInConfirming = 0;
    GPSOpenCl::ChannelState state = GPSOpenCl::ChannelState::Tracking;

    for (int i = 0; i < 80; i++)
    {
        state = GPSOpenCl::Channel::computeNextState(state, false, confirmProgress, lossProgress, blocksInConfirming,
                                                      50, 200, 100);
    }
    EXPECT_EQ(lossProgress, 80);
    EXPECT_EQ(state, GPSOpenCl::ChannelState::Tracking);

    for (int i = 0; i < 80; i++)
    {
        state = GPSOpenCl::Channel::computeNextState(state, true, confirmProgress, lossProgress, blocksInConfirming,
                                                      50, 200, 100);
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
        m_channel.m_svId = 1;
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

    int codeLength = m_settings.configuration.rawDataSettings.numberOfSamplesPerCode;
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
}
