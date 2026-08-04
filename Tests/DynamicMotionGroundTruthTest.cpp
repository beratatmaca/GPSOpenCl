#include "GPSOpenClApplication.h"
#include "GPSOpenClCommon.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClStructs.h"
#include "GroundTruthRecord.h"
#include "GroundTruthTestUtils.h"
#include "TestUtils.h"

#include "gtest/gtest.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <vector>

namespace GPSOpenClTest
{
TEST(DynamicMotionGroundTruthTest, AcquisitionAndTrackingMatchSimulatorTruthUnderMotion)
{
    std::string gpsSimBin = "../Tools/gps-sdr-sim/gps-sdr-sim";
    std::string navFile = "Tools/gps-sdr-sim/brdc0010.22n";
    std::string motionFile = "Tools/gps-sdr-sim/circle.csv";

    std::ifstream checkSim(gpsSimBin);
    if (!checkSim.is_open())
    {
        GTEST_SKIP() << "gps-sdr-sim binary not found at " << gpsSimBin << ". Skipping dynamic ground-truth test.";
        return;
    }

    std::string iqFile = "dynamic_motion_iq.bin";
    std::string truthFile = "dynamic_motion_truth.bin";

    std::string cmd = gpsSimBin + " -e " + navFile + " -u " + motionFile + " -s 4096000 -b 8 -d 2 -o " + iqFile +
        " -G " + truthFile + " > /dev/null 2>&1";
    int sysRet = std::system(cmd.c_str());
    ASSERT_EQ(sysRet, 0);

    std::vector<GroundTruthRecord> truth;
    ASSERT_TRUE(readGroundTruth(truthFile, &truth));
    ASSERT_FALSE(truth.empty());
    ASSERT_EQ(truth.front().structVersion, 2u);

    GPSOpenCl::ComplexFloatVector inputSignal;
    TestUtils::readFromFileBinaryIQ8(iqFile.c_str(), &inputSignal);
    ASSERT_GE(inputSignal.size(), 4096u);

    GPSOpenCl::Settings settings;
    settings.captureSettings();

    GPSOpenCl::Application app(settings.configuration);
    auto sink = std::make_shared<CapturingSink>();
    app.setSink(sink);

    int codeLength = settings.configuration.rawDataSettings.numberOfSamplesPerCode;
    ASSERT_GT(codeLength, 0);

    int blocksAvailable = static_cast<int>(inputSignal.size() / static_cast<size_t>(codeLength));
    int blocksToProcess = std::min(blocksAvailable, 450);
    ASSERT_GE(blocksToProcess, 2);

    for (int blockIdx = 0; blockIdx < blocksToProcess; blockIdx++)
    {
        auto start = inputSignal.begin() + static_cast<long>(blockIdx) * codeLength;
        auto end = start + codeLength;
        GPSOpenCl::ComplexFloatVector block(start, end);

        if (blockIdx == 0)
        {
            app.searchForSatellites(block);
        }
        app.trackSatellites(block);
    }

    ASSERT_FALSE(sink->acquisitionOutputs.empty());

    std::map<int32_t, GroundTruthRecord> firstTruthByPrn;
    for (const GroundTruthRecord &record : truth)
    {
        if (firstTruthByPrn.find(record.prn) == firstTruthByPrn.end())
        {
            firstTruthByPrn[record.prn] = record;
        }
    }

    int acquiredCount = 0;
    for (const GPSOpenCl::AcquisitionOutput &acq : sink->acquisitionOutputs)
    {
        if (!acq.isAcquired)
        {
            continue;
        }

        auto it = firstTruthByPrn.find(acq.prn);
        ASSERT_NE(it, firstTruthByPrn.end()) << "Acquired PRN " << acq.prn << " has no ground truth record";

        double dopplerDiff = std::fabs(acq.peakFrequency - it->second.trueDopplerHz);
        EXPECT_LT(dopplerDiff, 400.0) << "PRN " << acq.prn << " acquired Doppler " << acq.peakFrequency
                                      << " Hz too far from true Doppler " << it->second.trueDopplerHz << " Hz";
        acquiredCount++;
    }
    EXPECT_GE(acquiredCount, 10)
        << "Expected most of the scenario's real satellites to acquire under a moving receiver trajectory";

    ASSERT_FALSE(sink->trackingOutputs.empty());

    std::map<int32_t, GPSOpenCl::TrackingOutput> lastTrackingByPrn;
    for (const GPSOpenCl::TrackingOutput &trk : sink->trackingOutputs)
    {
        lastTrackingByPrn[trk.prn] = trk;
    }

    std::map<int32_t, GroundTruthRecord> lastTruthByPrn;
    for (const GroundTruthRecord &record : truth)
    {
        lastTruthByPrn[record.prn] = record;
    }

    int convergedCount = 0;
    for (const auto &entry : lastTrackingByPrn)
    {
        auto it = lastTruthByPrn.find(entry.first);
        if (it == lastTruthByPrn.end())
        {
            continue;
        }
        if (entry.second.channelState != 2)
        {
            continue;
        }

        double dopplerDiff = std::fabs(entry.second.carrierFreqHz - it->second.trueDopplerHz);
        EXPECT_LT(dopplerDiff, 50.0) << "PRN " << entry.first << " tracked Doppler " << entry.second.carrierFreqHz
                                     << " Hz too far from true Doppler " << it->second.trueDopplerHz << " Hz";
        convergedCount++;
    }
    EXPECT_GT(convergedCount, 0) << "Expected at least one channel to reach confirmed Tracking under motion";

    std::remove(iqFile.c_str());
    std::remove(truthFile.c_str());
}
}    // namespace GPSOpenClTest
