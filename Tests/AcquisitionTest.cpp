#include "../Source/GPSOpenClAcquisition.h"
#include "../Source/GPSOpenClChannel.h"
#include "../Source/GPSOpenClCode.h"
#include "../Source/GPSOpenClCommon.h"
#include "../Source/GPSOpenClGPUCompute.h"
#include "../Source/GPSOpenClSettings.h"
#include "../Source/GPSOpenClStructs.h"

#include "gtest/gtest.h"

#include <cmath>
#include <complex>

namespace GPSOpenClTest
{
namespace
{
GPSOpenCl::AcquisitionInput makeInput(int minDopplerHz, int maxDopplerHz, int spacingHz)
{
    GPSOpenCl::AcquisitionInput input{};
    input.acquisitionDopplerMinimum = minDopplerHz;
    input.acquisitionDopplerMaximum = maxDopplerHz;
    input.acquisitionDopplerSearchRange = spacingHz;
    input.samplingFrequencyHz = 4096000.0;
    input.numberOfSamplesPerCode = 4096;
    return input;
}
}    // namespace

TEST(AcquisitionTest, ReuseFactorMatchesBinResolutionRatioForAlignedSpacing)
{
    // 4096 samples at 4.096 MHz gives a 1000 Hz FFT bin resolution.
    GPSOpenCl::Acquisition halfBinSpacing(makeInput(-4000, 4000, 500));
    EXPECT_EQ(halfBinSpacing.getReuseFactor(), 2);

    GPSOpenCl::Acquisition fullBinSpacing(makeInput(-4000, 4000, 1000));
    EXPECT_EQ(fullBinSpacing.getReuseFactor(), 1);
}

TEST(AcquisitionTest, ReuseFactorFallsBackToPerBinFftForMisalignedSpacing)
{
    // 700 Hz does not divide the 1000 Hz bin resolution: every one of the 11 bins needs its own
    // forward FFT, because deriving them by circular shift would search wrong frequencies.
    GPSOpenCl::Acquisition misaligned(makeInput(-3500, 3500, 700));
    EXPECT_EQ(misaligned.getReuseFactor(), 11);

    // A spacing wider than the bin resolution cannot be represented by single-bin shifts either.
    GPSOpenCl::Acquisition coarse(makeInput(-4000, 4000, 2000));
    EXPECT_EQ(coarse.getReuseFactor(), 5);
}

TEST(AcquisitionTest, CorrelateFindsInjectedDopplerWithMisalignedSpacing)
{
    GPSOpenCl::Settings settings;
    settings.configuration.acquisitionInput.samplingFrequencyHz = 4096000.0f;
    settings.configuration.acquisitionInput.numberOfSamplesPerCode = 4096;
    settings.configuration.acquisitionInput.acquisitionDopplerMinimum = -3500;
    settings.configuration.acquisitionInput.acquisitionDopplerMaximum = 3500;
    settings.configuration.acquisitionInput.acquisitionDopplerSearchRange = 700;

    GPSOpenCl::Compute gpu;
    GPSOpenCl::Code code(settings.configuration);
    code.createLookupTable(&gpu);
    ASSERT_FALSE(code.upsampledCaCode.empty());

    GPSOpenCl::Acquisition acquisition(settings.configuration);

    // 1400 Hz sits on the 700 Hz search grid but not on the 1000 Hz FFT bin grid, so a
    // shift-derived spectrum could never represent it exactly.
    const double samplingFrequencyHz = 4096000.0;
    const double targetBinHz = 1400.0;
    const int sv = 5;

    GPSOpenCl::ComplexFloatVector input(4096);
    for (int n = 0; n < 4096; n++)
    {
        const double phase = -2.0 * M_PI * targetBinHz * static_cast<double>(n) / samplingFrequencyHz;
        const float chip = code.upsampledCaCode[sv - 1][static_cast<size_t>(n)];
        input[static_cast<size_t>(n)] =
            std::complex<float>(chip * static_cast<float>(std::cos(phase)), chip * static_cast<float>(std::sin(phase)));
    }

    GPSOpenCl::Channel channel;
    channel.svId = sv;
    acquisition.correlate(input, &gpu, &code, &channel);

    int peakIndex = 0;
    float peakValue = 0.0f;
    float peakFrequencyHz = 0.0f;
    float meanValue = 0.0f;
    float cn0 = 0.0f;
    float peakRatio = 0.0f;
    channel.getAcquisitionResults(&peakIndex, &peakValue, &peakFrequencyHz, &meanValue, &cn0, &peakRatio);

    EXPECT_NEAR(peakFrequencyHz, targetBinHz, 1.0f)
        << "Correlation peak found at " << peakFrequencyHz << " Hz instead of the injected " << targetBinHz << " Hz";
    EXPECT_EQ(peakIndex, 0) << "Zero-delay code should peak at code phase index 0";
    EXPECT_GT(cn0, 43.0f) << "Noiseless aligned signal should clear the acquisition C/N0 threshold";
}
}    // namespace GPSOpenClTest
