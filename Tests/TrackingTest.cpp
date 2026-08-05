#include "GPSOpenClTracking.h"

#include "GPSOpenClCode.h"
#include "GPSOpenClSettings.h"

#include "gtest/gtest.h"

#include <cmath>
#include <complex>

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

TEST(TrackingCarrierAidingTest, CodeFreqTracksDopplerScaledRateWithinFewBlocks)
{
    GPSOpenCl::Settings settings;
    settings.captureSettings();

    GPSOpenCl::Code code;
    code.setConfiguration(settings.configuration);

    const int prn = 1;
    const int svIndex = prn - 1;
    const int codeLength = settings.configuration.rawDataSettings.numberOfSamplesPerCode;
    const float samplingFreq = settings.configuration.rawDataSettings.samplingFrequency;
    ASSERT_GT(codeLength, 0);
    ASSERT_GT(samplingFreq, 0.0f);

    const double trueDopplerHz = 2500.0;
    const double trueCodeFreqHz = static_cast<double>(GPSOpenCl::GPS_CA_CODE_FREQUENCY_HZ) +
        trueDopplerHz / static_cast<double>(GPSOpenCl::GPS_L1_CARRIER_TO_CODE_RATIO);
    const double codePhaseStep = trueCodeFreqHz / static_cast<double>(samplingFreq);
    const double carrierPhaseStep = 2.0 * M_PI * trueDopplerHz / static_cast<double>(samplingFreq);

    GPSOpenCl::Tracking tracking(settings.configuration);
    tracking.initTrackingState(static_cast<float>(trueDopplerHz), 0.0f);

    double codePhase = 0.0;
    double carrierPhase = 0.0;
    GPSOpenCl::TrackingOutput output{};

    const int blocksToRun = 150;
    for (int block = 0; block < blocksToRun; block++)
    {
        GPSOpenCl::ComplexFloatVector input(static_cast<size_t>(codeLength));
        for (int i = 0; i < codeLength; i++)
        {
            int chipIdx = static_cast<int>(std::floor(codePhase)) % GPSOpenCl::GPS_CA_CODE_LENGTH;
            if (chipIdx < 0)
            {
                chipIdx += GPSOpenCl::GPS_CA_CODE_LENGTH;
            }
            float chip = static_cast<float>(code.m_caCode[svIndex][chipIdx]);
            std::complex<float> carrier(static_cast<float>(std::cos(carrierPhase)),
                                        static_cast<float>(std::sin(carrierPhase)));
            input[static_cast<size_t>(i)] = chip * carrier;

            codePhase = std::fmod(codePhase + codePhaseStep, static_cast<double>(GPSOpenCl::GPS_CA_CODE_LENGTH));
            carrierPhase = std::fmod(carrierPhase + carrierPhaseStep, 2.0 * M_PI);
        }

        GPSOpenCl::ComplexFloatVector promptOut;
        tracking.doWork(input, prn, &promptOut, 2);
        output = tracking.getTrackingOutput(prn);
    }

    EXPECT_NEAR(output.carrierFreqHz, trueDopplerHz, 5.0)
        << "Carrier loop failed to converge to the true Doppler within " << blocksToRun << " blocks";

    EXPECT_NEAR(output.codeFreqHz, trueCodeFreqHz, 0.3)
        << "Code loop failed to converge to the Doppler-scaled true code rate within " << blocksToRun
        << " blocks -- carrier aiding should make this fast even though the DLL's own 2 Hz bandwidth "
        << "alone would need far longer to settle";
}

TEST(TrackingFllDiscriminatorTest, PullsInFullBinAcquisitionError)
{
    const double freqHz = 375.0;
    const double theta = 2.0 * M_PI * freqHz * 0.001;

    const float error = GPSOpenCl::Tracking::computeFllError(1.0, 0.0, std::cos(theta), std::sin(theta));

    EXPECT_NEAR(error, freqHz, 0.5);
}

TEST(TrackingFllDiscriminatorTest, MeasuresSmallFrequencyOffset)
{
    const double freqHz = 50.0;
    const double theta = 2.0 * M_PI * freqHz * 0.001;

    const float error = GPSOpenCl::Tracking::computeFllError(1.0, 0.0, std::cos(theta), std::sin(theta));

    EXPECT_NEAR(error, freqHz, 0.5);
}

TEST(TrackingFllDiscriminatorTest, ZeroInputGivesZeroError)
{
    EXPECT_EQ(GPSOpenCl::Tracking::computeFllError(0.0, 0.0, 0.0, 0.0), 0.0f);
}
}
