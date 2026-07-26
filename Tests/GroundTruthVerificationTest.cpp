#include "GPSOpenClApplication.h"
#include "GPSOpenClCommon.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClSink.h"
#include "GPSOpenClStructs.h"
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
#include <vector>

namespace GPSOpenClTest
{
namespace
{
class CapturingSink : public GPSOpenCl::Sink
{
  public:
    std::vector<GPSOpenCl::AcquisitionOutput> acquisitionOutputs;
    std::vector<GPSOpenCl::TrackingOutput> trackingOutputs;
    std::vector<GPSOpenCl::NavDecoderOutput> navDecoderOutputs;
    std::vector<GPSOpenCl::PvtSolverOutput> pvtSolverOutputs;

    void publish(const std::string &identifier, const void *data, size_t size) override
    {
        if (identifier == "AcquisitionOutput" && size == sizeof(GPSOpenCl::AcquisitionOutput))
        {
            GPSOpenCl::AcquisitionOutput out;
            std::memcpy(&out, data, sizeof(out));
            acquisitionOutputs.push_back(out);
        }
        else if (identifier == "TrackingOutput" && size == sizeof(GPSOpenCl::TrackingOutput))
        {
            GPSOpenCl::TrackingOutput out;
            std::memcpy(&out, data, sizeof(out));
            trackingOutputs.push_back(out);
        }
        else if (identifier == "NavDecoderOutput" && size == sizeof(GPSOpenCl::NavDecoderOutput))
        {
            GPSOpenCl::NavDecoderOutput out;
            std::memcpy(&out, data, sizeof(out));
            navDecoderOutputs.push_back(out);
        }
        else if (identifier == "PvtSolverOutput" && size == sizeof(GPSOpenCl::PvtSolverOutput))
        {
            GPSOpenCl::PvtSolverOutput out;
            std::memcpy(&out, data, sizeof(out));
            pvtSolverOutputs.push_back(out);
        }
    }
};

/** @brief Merge per-subframe NavDecoderOutput captures into an accumulated ephemeris, mirroring Channel::updateNavigation. */
class EphemerisAccumulator
{
  public:
    void ingest(const GPSOpenCl::NavDecoderOutput &out)
    {
        if (!out.isValid)
        {
            return;
        }

        GPSOpenCl::GpsEphemeris &ephem = m_ephemerisByPrn[out.svId];
        ephem.svId = out.svId;

        switch (out.subframeId)
        {
        case 1:
            ephem.weekNumber = out.weekNumber;
            ephem.toc = out.toc;
            ephem.af0 = out.af0;
            ephem.af1 = out.af1;
            ephem.af2 = out.af2;
            m_seenMask[out.svId] |= 0x1;
            break;
        case 2:
            ephem.toe = out.toe;
            ephem.sqrtA = out.sqrtA;
            ephem.e = out.e;
            ephem.M0 = out.M0;
            ephem.deltaN = out.deltaN;
            ephem.Cuc = out.Cuc;
            ephem.Cus = out.Cus;
            ephem.Crs = out.Crs;
            m_seenMask[out.svId] |= 0x2;
            break;
        case 3:
            ephem.i0 = out.i0;
            ephem.idot = out.idot;
            ephem.omega0 = out.omega0;
            ephem.omegaDot = out.omegaDot;
            ephem.omega = out.omega;
            ephem.Cic = out.Cic;
            ephem.Cis = out.Cis;
            ephem.Crc = out.Crc;
            m_seenMask[out.svId] |= 0x4;
            break;
        default:
            break;
        }
    }

    bool isComplete(int prn) const
    {
        auto it = m_seenMask.find(prn);
        return it != m_seenMask.end() && (it->second & 0x7) == 0x7;
    }

    const GPSOpenCl::GpsEphemeris &get(int prn) const { return m_ephemerisByPrn.at(prn); }

    std::vector<int> completedPrns() const
    {
        std::vector<int> prns;
        for (const auto &entry : m_seenMask)
        {
            if ((entry.second & 0x7) == 0x7)
            {
                prns.push_back(entry.first);
            }
        }
        return prns;
    }

  private:
    std::map<int, GPSOpenCl::GpsEphemeris> m_ephemerisByPrn;
    std::map<int, uint8_t> m_seenMask;
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
} // namespace

TEST(GroundTruthVerificationTest, AcquisitionAndTrackingMatchSimulatorTruth)
{
    std::string gpsSimBin = "../Tools/gps-sdr-sim/gps-sdr-sim";
    std::string navFile = "Tools/gps-sdr-sim/brdc0010.22n";

    std::ifstream checkSim(gpsSimBin);
    if (!checkSim.is_open())
    {
        GTEST_SKIP() << "gps-sdr-sim binary not found at " << gpsSimBin << ". Skipping ground-truth verification test.";
        return;
    }

    std::string iqFile = "ground_truth_iq.bin";
    std::string truthFile = "ground_truth_records.bin";

    std::string cmd = gpsSimBin + " -e " + navFile +
                       " -l 48.1173,11.5167,545.4 -s 4096000 -b 8 -d 2 -o " + iqFile +
                       " -G " + truthFile + " > /dev/null 2>&1";
    int sysRet = std::system(cmd.c_str());
    ASSERT_EQ(sysRet, 0);

    std::vector<GroundTruthRecord> truth;
    ASSERT_TRUE(readGroundTruth(truthFile, &truth));
    ASSERT_FALSE(truth.empty());
    ASSERT_EQ(truth.front().structVersion, 1u);

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
    EXPECT_GE(acquiredCount, 10) << "Expected most of the scenario's real satellites to acquire";

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
    EXPECT_GT(convergedCount, 0) << "Expected at least one channel to reach confirmed Tracking within the test window";

    std::remove(iqFile.c_str());
    std::remove(truthFile.c_str());
}

TEST(GroundTruthVerificationTest, FullEphemerisAndPvtMatchSimulatorTruth)
{
    if (std::getenv("GPSOPENCL_RUN_INTEGRATION_TESTS") == nullptr)
    {
        GTEST_SKIP() << "Slow full-ephemeris/PVT verification skipped by default. "
                     << "Set GPSOPENCL_RUN_INTEGRATION_TESTS=1 to run it (takes several minutes).";
        return;
    }

    std::string gpsSimBin = "../Tools/gps-sdr-sim/gps-sdr-sim";
    std::string navFile = "Tools/gps-sdr-sim/brdc0010.22n";

    std::ifstream checkSim(gpsSimBin);
    if (!checkSim.is_open())
    {
        GTEST_SKIP() << "gps-sdr-sim binary not found at " << gpsSimBin << ". Skipping ground-truth verification test.";
        return;
    }

    std::string iqFile = "ground_truth_iq_long.bin";
    std::string cmd = gpsSimBin + " -e " + navFile +
                       " -l 48.1173,11.5167,545.4 -s 4096000 -b 8 -d 32 -o " + iqFile + " > /dev/null 2>&1";
    int sysRet = std::system(cmd.c_str());
    ASSERT_EQ(sysRet, 0);

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

    for (int blockIdx = 0; blockIdx < blocksAvailable; blockIdx++)
    {
        auto start = inputSignal.begin() + static_cast<long>(blockIdx) * codeLength;
        auto end = start + codeLength;
        GPSOpenCl::ComplexFloatVector block(start, end);
        app.processBlock(block, static_cast<uint32_t>(blockIdx));
    }

    EphemerisAccumulator accumulator;
    for (const GPSOpenCl::NavDecoderOutput &out : sink->navDecoderOutputs)
    {
        accumulator.ingest(out);
    }

    std::vector<int> completedPrns = accumulator.completedPrns();
    ASSERT_FALSE(completedPrns.empty()) << "Expected at least one satellite to decode a complete ephemeris "
                                        << "(subframes 1-3) within " << blocksAvailable << " blocks";

    int verifiedCount = 0;
    for (int prn : completedPrns)
    {
        GPSOpenCl::GpsEphemeris truthEphem{};
        if (!RinexNavParser::findEphemeris(navFile, prn, &truthEphem))
        {
            continue;
        }

        const GPSOpenCl::GpsEphemeris &decoded = accumulator.get(prn);
        const double tolerance = 0.001;

        TestUtils::compareRealResults(static_cast<float>(decoded.sqrtA), static_cast<float>(truthEphem.sqrtA), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.e), static_cast<float>(truthEphem.e), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.M0), static_cast<float>(truthEphem.M0), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.deltaN), static_cast<float>(truthEphem.deltaN), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.toe), static_cast<float>(truthEphem.toe), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.i0), static_cast<float>(truthEphem.i0), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.idot), static_cast<float>(truthEphem.idot), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.omega0), static_cast<float>(truthEphem.omega0), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.omega), static_cast<float>(truthEphem.omega), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.omegaDot), static_cast<float>(truthEphem.omegaDot), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.Cuc), static_cast<float>(truthEphem.Cuc), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.Cus), static_cast<float>(truthEphem.Cus), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.Crc), static_cast<float>(truthEphem.Crc), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.Crs), static_cast<float>(truthEphem.Crs), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.Cic), static_cast<float>(truthEphem.Cic), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.Cis), static_cast<float>(truthEphem.Cis), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.af0), static_cast<float>(truthEphem.af0), tolerance);
        TestUtils::compareRealResults(static_cast<float>(decoded.af1), static_cast<float>(truthEphem.af1), tolerance);
        EXPECT_EQ(decoded.weekNumber, truthEphem.weekNumber) << "PRN " << prn;

        verifiedCount++;
    }
    EXPECT_GT(verifiedCount, 0) << "No completed ephemeris had a matching RINEX record to verify against";

    ASSERT_FALSE(sink->pvtSolverOutputs.empty()) << "Expected at least one PVT solution attempt";

    // gps-sdr-sim's own printed ECEF for lat=48.1173,lon=11.5167,alt=545.4 (cross-referenced at scenario startup,
    // not re-derived here to avoid introducing a second, independently-unverified WGS-84 conversion).
    const double trueEcefX = 4180483.4;
    const double trueEcefY = 851798.0;
    const double trueEcefZ = 4725999.8;
    const double positionToleranceMeters = 50.0;

    bool foundValidPvt = false;
    for (const GPSOpenCl::PvtSolverOutput &pvt : sink->pvtSolverOutputs)
    {
        if (!pvt.isValid)
        {
            continue;
        }

        double dx = pvt.ecefX - trueEcefX;
        double dy = pvt.ecefY - trueEcefY;
        double dz = pvt.ecefZ - trueEcefZ;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        EXPECT_LT(distance, positionToleranceMeters) << "PVT solution " << distance
                                                      << " m from true receiver position";
        foundValidPvt = true;
    }
    EXPECT_TRUE(foundValidPvt) << "Expected at least one valid PVT solution within " << blocksAvailable << " blocks";

    std::remove(iqFile.c_str());
}
}
