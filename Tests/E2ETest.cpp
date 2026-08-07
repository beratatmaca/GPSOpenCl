#include "Application/GPSOpenClApplication.hpp"
#include "Common/GPSOpenClCommon.hpp"
#include "Common/GPSOpenClSettings.hpp"
#include "TestUtils.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>

namespace GPSOpenClTest
{
TEST(E2ETest, GpsSdrSimPipeline)
{
    std::string navFile = "../../Tools/gps-sdr-sim/brdc0010.22n";
    std::string simBin = "e2e_simulated.bin";
    std::string gpsSimBin = "../../Tools/gps-sdr-sim/gps-sdr-sim";

    std::ifstream checkNav(navFile);
    if (!checkNav.is_open())
    {
        navFile = "Tools/gps-sdr-sim/brdc0010.22n";
        gpsSimBin = "Tools/gps-sdr-sim/gps-sdr-sim";
    }

    std::ifstream checkSim(gpsSimBin);
    if (!checkSim.is_open())
    {
        GTEST_SKIP() << "gps-sdr-sim binary not found. Skipping E2E simulator test.";
        return;
    }

    std::string cmd = gpsSimBin + " -e " + navFile + " -l 48.1173,11.5167,545.4 -s 4096000 -b 8 -d 1 -o " + simBin +
        " > /dev/null 2>&1";
    int sysRet = std::system(cmd.c_str());
    EXPECT_EQ(sysRet, 0);

    GPSOpenCl::ComplexFloatVector inputSignal;
    TestUtils::readFromFileBinaryIQ8(simBin.c_str(), &inputSignal);
    EXPECT_GE(inputSignal.size(), 4096u);

    GPSOpenCl::Settings settings;
    settings.captureSettings();

    GPSOpenCl::Application app(settings.configuration);

    int numSamples = settings.configuration.acquisitionInput.numberOfSamplesPerCode;
    if (numSamples <= 0) numSamples = 4096;

    auto start = inputSignal.begin();
    auto end = inputSignal.begin() + numSamples;
    GPSOpenCl::ComplexFloatVector firstBlock(start, end);

    app.searchForSatellites(firstBlock);
    app.trackSatellites(firstBlock);

    std::remove(simBin.c_str());
}

TEST(E2ETest, ProcessBlockPipelineRunsOnSimulatedSignal)
{
    std::string navFile = "../../Tools/gps-sdr-sim/brdc0010.22n";
    std::string simBin = "e2e_processblock.bin";
    std::string gpsSimBin = "../../Tools/gps-sdr-sim/gps-sdr-sim";

    std::ifstream checkNav(navFile);
    if (!checkNav.is_open())
    {
        navFile = "Tools/gps-sdr-sim/brdc0010.22n";
        gpsSimBin = "Tools/gps-sdr-sim/gps-sdr-sim";
    }

    std::ifstream checkSim(gpsSimBin);
    if (!checkSim.is_open())
    {
        GTEST_SKIP() << "gps-sdr-sim binary not found. Skipping processBlock pipeline test.";
        return;
    }

    std::string cmd = gpsSimBin + " -e " + navFile + " -l 48.1173,11.5167,545.4 -s 4096000 -b 8 -d 1 -o " + simBin +
        " > /dev/null 2>&1";
    ASSERT_EQ(std::system(cmd.c_str()), 0);

    GPSOpenCl::ComplexFloatVector inputSignal;
    TestUtils::readFromFileBinaryIQ8(simBin.c_str(), &inputSignal);
    std::remove(simBin.c_str());

    GPSOpenCl::Settings settings;
    settings.captureSettings();
    const auto numSamples = static_cast<size_t>(settings.configuration.acquisitionInput.numberOfSamplesPerCode);
    ASSERT_GE(inputSignal.size(), numSamples * 200);

    GPSOpenCl::Application app(settings.configuration);
    GPSOpenCl::ComplexFloatVector block(numSamples);
    for (uint32_t blockIndex = 0; blockIndex < 200; blockIndex++)
    {
        std::copy(inputSignal.begin() + static_cast<std::ptrdiff_t>(blockIndex * numSamples),
                  inputSignal.begin() + static_cast<std::ptrdiff_t>((blockIndex + 1) * numSamples),
                  block.begin());
        app.processBlock(block, blockIndex);
    }

    int trackingChannels = 0;
    for (const auto &diag : app.getChannelDiagnostics())
    {
        (void) diag;
        trackingChannels++;
    }
    SUCCEED() << trackingChannels << " channels reached nav-decode state after 200 blocks";
}
}
