#include "GPSOpenClAcquisition.h"
#include "GPSOpenClChannel.h"
#include "GPSOpenClCode.h"
#include "GPSOpenClCommon.h"
#include "GPSOpenClGPUCompute.h"
#include "GPSOpenClNavigationDecoder.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClSink.h"
#include "GroundTruthRecord.h"
#include "RinexNavParser.h"
#include "TestUtils.h"

#include "gtest/gtest.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <vector>

namespace GPSOpenClTest
{
namespace
{
class CapturingSink : public GPSOpenCl::Sink
{
  public:
    GPSOpenCl::TrackingOutput lastTracking{};
    bool haveTracking = false;

    void publish(const std::string &identifier, const void *data, size_t size) override
    {
        if (identifier == "TrackingOutput" && size == sizeof(GPSOpenCl::TrackingOutput))
        {
            std::memcpy(&lastTracking, data, sizeof(lastTracking));
            haveTracking = true;
        }
    }
};

bool readGroundTruth(const std::string &path, std::vector<GroundTruthRecord> *records)
{
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open())
    {
        return false;
    }

    GroundTruthRecord record;
    while (file.read(reinterpret_cast<char *>(&record), sizeof(record)))
    {
        records->push_back(record);
    }
    return true;
}

/** @brief PRN with the highest elevation seen in the ground-truth records (strongest, most reliable signal). */
int strongestPrn(const std::vector<GroundTruthRecord> &truth)
{
    std::map<int32_t, double> maxElevationByPrn;
    for (const GroundTruthRecord &r : truth)
    {
        double &best = maxElevationByPrn[r.prn];
        best = std::max(best, r.trueElevationDeg);
    }

    int bestPrn = 0;
    double bestElevation = -1.0;
    for (const auto &entry : maxElevationByPrn)
    {
        if (entry.second > bestElevation)
        {
            bestElevation = entry.second;
            bestPrn = entry.first;
        }
    }
    return bestPrn;
}

GroundTruthRecord firstRecordForPrn(const std::vector<GroundTruthRecord> &truth, int prn)
{
    for (const GroundTruthRecord &r : truth)
    {
        if (r.prn == prn)
        {
            return r;
        }
    }
    return GroundTruthRecord{};
}

GroundTruthRecord lastRecordForPrn(const std::vector<GroundTruthRecord> &truth, int prn)
{
    GroundTruthRecord last{};
    for (const GroundTruthRecord &r : truth)
    {
        if (r.prn == prn)
        {
            last = r;
        }
    }
    return last;
}
} // namespace

TEST(SingleChannelGroundTruthTest, AcquisitionAndTrackingConvergeForOneChannel)
{
    std::string gpsSimBin = "../Tools/gps-sdr-sim/gps-sdr-sim";
    std::string navFile = "Tools/gps-sdr-sim/brdc0010.22n";

    std::ifstream checkSim(gpsSimBin);
    if (!checkSim.is_open())
    {
        GTEST_SKIP() << "gps-sdr-sim binary not found at " << gpsSimBin << ". Skipping single-channel test.";
        return;
    }

    std::string iqFile = "single_channel_iq.bin";
    std::string truthFile = "single_channel_truth.bin";
    std::string cmd = gpsSimBin + " -e " + navFile +
                       " -l 48.1173,11.5167,545.4 -s 4096000 -b 8 -d 2 -o " + iqFile + " -G " + truthFile +
                       " > /dev/null 2>&1";
    int sysRet = std::system(cmd.c_str());
    ASSERT_EQ(sysRet, 0);

    std::vector<GroundTruthRecord> truth;
    ASSERT_TRUE(readGroundTruth(truthFile, &truth));
    ASSERT_FALSE(truth.empty());

    int prn = strongestPrn(truth);
    ASSERT_GT(prn, 0) << "No satellite found in ground truth";
    GroundTruthRecord firstTruth = firstRecordForPrn(truth, prn);

    GPSOpenCl::ComplexFloatVector inputSignal;
    TestUtils::readFromFileBinaryIQ8(iqFile.c_str(), &inputSignal);
    ASSERT_GE(inputSignal.size(), 4096u);

    GPSOpenCl::Settings settings;
    settings.captureSettings();

    GPSOpenCl::Compute gpu;
    GPSOpenCl::Code code(settings.configuration);
    code.createLookupTable(&gpu);

    GPSOpenCl::Acquisition acquisition(settings.configuration);
    GPSOpenCl::Channel channel;
    channel.m_svId = prn;

    auto sink = std::make_shared<CapturingSink>();
    channel.setSink(sink);

    int codeLength = settings.configuration.rawDataSettings.numberOfSamplesPerCode;
    ASSERT_GT(codeLength, 0);
    ASSERT_GE(inputSignal.size(), static_cast<size_t>(codeLength));

    GPSOpenCl::ComplexFloatVector firstBlock(inputSignal.begin(), inputSignal.begin() + codeLength);
    acquisition.correlate(firstBlock, &gpu, &code, &channel);

    int peakIndex = 0;
    float peakValue = 0.0f, peakFreq = 0.0f, meanValue = 0.0f, cn0 = 0.0f, peakRatio = 0.0f;
    channel.getAcquisitionResults(&peakIndex, &peakValue, &peakFreq, &meanValue, &cn0, &peakRatio);

    float dopplerHz = -peakFreq;
    double dopplerDiff = std::fabs(static_cast<double>(dopplerHz) - firstTruth.trueDopplerHz);
    EXPECT_LT(dopplerDiff, 400.0) << "PRN " << prn << " acquired Doppler " << dopplerHz
                                  << " Hz too far from true Doppler " << firstTruth.trueDopplerHz << " Hz";

    float numSamplesFloat = static_cast<float>(codeLength);
    int reflectedPeakIndex = (codeLength - peakIndex) % codeLength;
    float codePhaseChips = (static_cast<float>(reflectedPeakIndex) / numSamplesFloat) * GPSOpenCl::GPS_CA_CODE_LENGTH;

    channel.setAcquired(true);
    channel.initTracking(settings.configuration, dopplerHz, codePhaseChips);

    int totalBlocks = static_cast<int>(inputSignal.size() / static_cast<size_t>(codeLength));
    int blocksToRun = std::min(totalBlocks - 1, 450);
    ASSERT_GE(blocksToRun, 2);

    for (int b = 1; b <= blocksToRun; b++)
    {
        GPSOpenCl::ComplexFloatVector block(inputSignal.begin() + static_cast<long>(b) * codeLength,
                                            inputSignal.begin() + static_cast<long>(b + 1) * codeLength);
        channel.trackBlock(block);
    }

    ASSERT_TRUE(sink->haveTracking) << "Expected at least one TrackingOutput to be published";
    EXPECT_EQ(channel.getState(), GPSOpenCl::ChannelState::Tracking)
        << "Expected the channel to reach confirmed Tracking within " << blocksToRun << " blocks";

    GroundTruthRecord lastTruth = lastRecordForPrn(truth, prn);
    double trackedDopplerDiff = std::fabs(sink->lastTracking.carrierFreqHz - lastTruth.trueDopplerHz);
    EXPECT_LT(trackedDopplerDiff, 50.0) << "PRN " << prn << " tracked Doppler " << sink->lastTracking.carrierFreqHz
                                        << " Hz too far from true Doppler " << lastTruth.trueDopplerHz << " Hz";
    EXPECT_GT(sink->lastTracking.carrierLockIndicator, 0.7)
        << "Expected carrier lock indicator near 1.0 once tracking is confirmed";

    std::remove(iqFile.c_str());
    std::remove(truthFile.c_str());
}

TEST(SingleChannelGroundTruthTest, FullEphemerisMatchesRinexForOneChannel)
{
    if (std::getenv("GPSOPENCL_RUN_INTEGRATION_TESTS") == nullptr)
    {
        GTEST_SKIP() << "Slow single-channel full-ephemeris verification skipped by default. "
                     << "Set GPSOPENCL_RUN_INTEGRATION_TESTS=1 to run it (takes about a minute).";
        return;
    }

    std::string gpsSimBin = "../Tools/gps-sdr-sim/gps-sdr-sim";
    std::string navFile = "Tools/gps-sdr-sim/brdc0010.22n";

    std::ifstream checkSim(gpsSimBin);
    if (!checkSim.is_open())
    {
        GTEST_SKIP() << "gps-sdr-sim binary not found at " << gpsSimBin << ". Skipping single-channel test.";
        return;
    }

    std::string iqFile = "single_channel_iq_long.bin";
    std::string truthFile = "single_channel_truth_long.bin";

    // Same reasoning as the multi-channel Tier 2 test: subframe 1's preamble falls at the scenario's
    // exact start, before acquisition/confirm can lock on to catch it, and only reappears ~30s later
    // in the second cycle. 60s covers two full cycles with margin - less than the multi-channel
    // test's 90s since this picks the single strongest (most reliable) satellite, not the weakest.
    std::string cmd = gpsSimBin + " -e " + navFile +
                       " -l 48.1173,11.5167,545.4 -s 4096000 -b 8 -d 60 -o " + iqFile + " -G " + truthFile +
                       " > /dev/null 2>&1";
    int sysRet = std::system(cmd.c_str());
    ASSERT_EQ(sysRet, 0);

    std::vector<GroundTruthRecord> truth;
    ASSERT_TRUE(readGroundTruth(truthFile, &truth));
    ASSERT_FALSE(truth.empty());

    int prn = strongestPrn(truth);
    ASSERT_GT(prn, 0) << "No satellite found in ground truth";

    GPSOpenCl::ComplexFloatVector inputSignal;
    TestUtils::readFromFileBinaryIQ8(iqFile.c_str(), &inputSignal);
    ASSERT_GE(inputSignal.size(), 4096u);

    GPSOpenCl::Settings settings;
    settings.captureSettings();

    GPSOpenCl::Compute gpu;
    GPSOpenCl::Code code(settings.configuration);
    code.createLookupTable(&gpu);

    GPSOpenCl::Acquisition acquisition(settings.configuration);
    GPSOpenCl::Channel channel;
    channel.m_svId = prn;

    int codeLength = settings.configuration.rawDataSettings.numberOfSamplesPerCode;
    ASSERT_GT(codeLength, 0);
    ASSERT_GE(inputSignal.size(), static_cast<size_t>(codeLength));

    GPSOpenCl::ComplexFloatVector firstBlock(inputSignal.begin(), inputSignal.begin() + codeLength);
    acquisition.correlate(firstBlock, &gpu, &code, &channel);

    int peakIndex = 0;
    float peakValue = 0.0f, peakFreq = 0.0f, meanValue = 0.0f, cn0 = 0.0f, peakRatio = 0.0f;
    channel.getAcquisitionResults(&peakIndex, &peakValue, &peakFreq, &meanValue, &cn0, &peakRatio);
    float dopplerHz = -peakFreq;

    float numSamplesFloat = static_cast<float>(codeLength);
    int reflectedPeakIndex = (codeLength - peakIndex) % codeLength;
    float codePhaseChips = (static_cast<float>(reflectedPeakIndex) / numSamplesFloat) * GPSOpenCl::GPS_CA_CODE_LENGTH;

    channel.setAcquired(true);
    channel.initTracking(settings.configuration, dopplerHz, codePhaseChips);

    GPSOpenCl::NavigationDecoder decoder;
    int totalBlocks = static_cast<int>(inputSignal.size() / static_cast<size_t>(codeLength));

    for (int b = 1; b < totalBlocks; b++)
    {
        GPSOpenCl::ComplexFloatVector block(inputSignal.begin() + static_cast<long>(b) * codeLength,
                                            inputSignal.begin() + static_cast<long>(b + 1) * codeLength);
        channel.trackBlock(block);

        if (channel.getState() == GPSOpenCl::ChannelState::Tracking)
        {
            channel.updateNavigation(decoder);
        }
    }

    ASSERT_TRUE(channel.hasCompleteEphemeris())
        << "Expected PRN " << prn << " to decode a complete ephemeris (subframes 1-3) within " << totalBlocks
        << " blocks";

    GPSOpenCl::GpsEphemeris truthEphem{};
    ASSERT_TRUE(RinexNavParser::findEphemeris(navFile, prn, &truthEphem)) << "No matching RINEX record for PRN "
                                                                          << prn;

    const GPSOpenCl::GpsEphemeris &decoded = channel.getAccumulatedEphemeris();
    const double tolerance = 0.001;

    TestUtils::compareRealResults(static_cast<float>(decoded.sqrtA), static_cast<float>(truthEphem.sqrtA), tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.e), static_cast<float>(truthEphem.e), tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.M0), static_cast<float>(truthEphem.M0), tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.deltaN), static_cast<float>(truthEphem.deltaN),
                                  tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.toe), static_cast<float>(truthEphem.toe), tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.i0), static_cast<float>(truthEphem.i0), tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.idot), static_cast<float>(truthEphem.idot), tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.omega0), static_cast<float>(truthEphem.omega0),
                                  tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.omega), static_cast<float>(truthEphem.omega),
                                  tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.omegaDot), static_cast<float>(truthEphem.omegaDot),
                                  tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.Cuc), static_cast<float>(truthEphem.Cuc), tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.Cus), static_cast<float>(truthEphem.Cus), tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.Crc), static_cast<float>(truthEphem.Crc), tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.Crs), static_cast<float>(truthEphem.Crs), tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.Cic), static_cast<float>(truthEphem.Cic), tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.Cis), static_cast<float>(truthEphem.Cis), tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.af0), static_cast<float>(truthEphem.af0), tolerance);
    TestUtils::compareRealResults(static_cast<float>(decoded.af1), static_cast<float>(truthEphem.af1), tolerance);
    // Subframe 1's broadcast WN field is a raw 10-bit (mod-1024) value per ICD-GPS-200, while RINEX
    // stores the un-rolled continuous GPS week - reduce the truth value the same way before comparing.
    EXPECT_EQ(decoded.weekNumber, truthEphem.weekNumber % 1024) << "PRN " << prn;

    std::remove(iqFile.c_str());
    std::remove(truthFile.c_str());
}
} // namespace GPSOpenClTest
