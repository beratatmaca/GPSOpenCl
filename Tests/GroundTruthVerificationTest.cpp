#include "Application/GPSOpenClApplication.hpp"
#include "Common/GPSOpenClCommon.hpp"
#include "Common/GPSOpenClSettings.hpp"
#include "Common/GPSOpenClStructs.hpp"
#include "GroundTruthRecord.hpp"
#include "GroundTruthTestUtils.hpp"
#include "RinexNavParser.hpp"
#include "TestUtils.hpp"

#include "gtest/gtest.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <map>
#include <vector>

namespace GPSOpenClTest
{
namespace
{
/** @brief Merge per-subframe NavDecoderOutput captures into an accumulated ephemeris, mirroring
 * Channel::updateNavigation. */
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
                ephem.tgd = out.tgd;
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

const GroundTruthRecord *
    findClosestRecord(const std::vector<GroundTruthRecord> &records, double targetTimeSec, double *outDt)
{
    auto lowerBound = std::lower_bound(records.begin(),
                                       records.end(),
                                       targetTimeSec,
                                       [](const GroundTruthRecord &record, double t) { return record.gpsTimeSec < t; });

    const GroundTruthRecord *closest = nullptr;
    double closestDt = 1e9;
    if (lowerBound != records.end())
    {
        closest = &(*lowerBound);
        closestDt = std::fabs(lowerBound->gpsTimeSec - targetTimeSec);
    }
    if (lowerBound != records.begin())
    {
        auto prev = std::prev(lowerBound);
        const double dtPrev = std::fabs(prev->gpsTimeSec - targetTimeSec);
        if (dtPrev < closestDt)
        {
            closest = &(*prev);
            closestDt = dtPrev;
        }
    }
    *outDt = closestDt;
    return closest;
}
}    // namespace

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

    std::string cmd = gpsSimBin + " -e " + navFile + " -l 48.1173,11.5167,545.4 -s 4096000 -b 8 -d 2 -o " + iqFile +
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

    int codeLength = settings.configuration.acquisitionInput.numberOfSamplesPerCode;
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

        double dopplerDiff = std::fabs(acq.peakFrequencyHz - it->second.trueDopplerHz);
        EXPECT_LT(dopplerDiff, 400.0) << "PRN " << acq.prn << " acquired Doppler " << acq.peakFrequencyHz
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

    // Subframes cycle 1-2-3-4-5 every 30s on a fixed GPS-time schedule shared by every satellite in
    // the scenario. Since the scenario's start time is frame-aligned, subframe 1's preamble falls at
    // the very first instant, before acquisition/confirm can lock on to catch it - every satellite
    // misses it in the first cycle and only catches it ~30s later in the second. PVT additionally
    // needs 4 satellites with complete ephemeris simultaneously; the weaker ones (~15 dB-Hz below the
    // strongest) need multiple clean cycles to get every word's parity right. 180s (six full cycles)
    // gives them a realistic chance; empirically 45-90s only ever got the single strongest satellite.
    std::string iqFile = "ground_truth_iq_long.bin";
    std::string truthFile = "ground_truth_records_long.bin";
    std::string cmd = gpsSimBin + " -e " + navFile + " -l 48.1173,11.5167,545.4 -s 4096000 -b 8 -d 180 -o " + iqFile +
        " -G " + truthFile + " > /dev/null 2>&1";
    int sysRet = std::system(cmd.c_str());
    ASSERT_EQ(sysRet, 0);

    std::vector<GroundTruthRecord> truth;
    ASSERT_TRUE(readGroundTruth(truthFile, &truth));
    ASSERT_FALSE(truth.empty());

    std::map<int, std::vector<GroundTruthRecord>> truthByPrn;
    for (const GroundTruthRecord &record : truth)
    {
        truthByPrn[record.prn].push_back(record);
    }

    GPSOpenCl::ComplexFloatVector inputSignal;
    TestUtils::readFromFileBinaryIQ8(iqFile.c_str(), &inputSignal);
    ASSERT_GE(inputSignal.size(), 4096u);

    GPSOpenCl::Settings settings;
    settings.captureSettings();

    GPSOpenCl::Application app(settings.configuration);
    auto sink = std::make_shared<CapturingSink>();
    app.setSink(sink);

    int codeLength = settings.configuration.acquisitionInput.numberOfSamplesPerCode;
    ASSERT_GT(codeLength, 0);

    int blocksAvailable = static_cast<int>(inputSignal.size() / static_cast<size_t>(codeLength));

    const double c = 299792458.0;
    std::map<int, double> maxErrorByPrn;
    std::map<int, double> sumErrorByPrn;
    std::map<int, int> countByPrn;
    std::map<int, bool> loggedByPrn;

    // Both the measured pseudorange and the simulator's truePseudorangeM are referenced to the
    // satellite's own clock, so they compare directly with no clock correction. The receiver's
    // synthesized common epoch legitimately carries an arbitrary offset (absorbed by the clock-bias
    // state in the solver), so errors are judged relative to the per-epoch mean across satellites.
    std::vector<std::pair<int, double>> epochDiffs;
    for (int blockIdx = 0; blockIdx < blocksAvailable; blockIdx++)
    {
        auto start = inputSignal.begin() + static_cast<long>(blockIdx) * codeLength;
        auto end = start + codeLength;
        GPSOpenCl::ComplexFloatVector block(start, end);
        app.processBlock(block, static_cast<uint32_t>(blockIdx));

        epochDiffs.clear();
        for (const GPSOpenCl::Application::PseudorangeSample &sample : app.getLastPseudorangeSamples())
        {
            auto prnTruth = truthByPrn.find(sample.svId);
            if (prnTruth == truthByPrn.end())
            {
                continue;
            }

            const double receiverTimeSec = sample.transmitTimeSeconds + (sample.measuredPseudorangeMeters / c);
            double closestDt = 1e9;
            const GroundTruthRecord *closest = findClosestRecord(prnTruth->second, receiverTimeSec, &closestDt);

            if (closest != nullptr && closestDt < 0.002)
            {
                epochDiffs.emplace_back(sample.svId, sample.measuredPseudorangeMeters - closest->truePseudorangeM);
            }
        }

        if (epochDiffs.size() < 2)
        {
            continue;
        }

        double meanDiff = 0.0;
        for (const auto &entry : epochDiffs)
        {
            meanDiff += entry.second;
        }
        meanDiff /= static_cast<double>(epochDiffs.size());

        for (const auto &entry : epochDiffs)
        {
            const double err = std::fabs(entry.second - meanDiff);
            if (!loggedByPrn[entry.first] || (blockIdx % 20'000) == 0)
            {
                loggedByPrn[entry.first] = true;
                std::cerr << "DEBUG prdiff PRN " << entry.first << " blockIdx=" << blockIdx
                          << " diffMeters=" << entry.second << " relErrMeters=" << (entry.second - meanDiff) << '\n';
            }
            maxErrorByPrn[entry.first] = std::max(maxErrorByPrn[entry.first], err);
            sumErrorByPrn[entry.first] += err;
            countByPrn[entry.first]++;
        }
    }

    int pseudorangeVerifiedCount = 0;
    double maxPseudorangeErrorMeters = 0.0;
    for (const auto &entry : countByPrn)
    {
        const int prn = entry.first;
        const int count = entry.second;
        const double meanErr = sumErrorByPrn[prn] / count;
        const double maxErr = maxErrorByPrn[prn];
        std::cerr << "DEBUG pseudorange PRN " << prn << " samples=" << count << " meanErrorMeters=" << meanErr
                  << " maxErrorMeters=" << maxErr << '\n';
        pseudorangeVerifiedCount += count;
        maxPseudorangeErrorMeters = std::max(maxPseudorangeErrorMeters, maxErr);
    }

    std::cerr << "DEBUG pseudorangeVerifiedCount=" << pseudorangeVerifiedCount
              << " maxPseudorangeErrorMeters=" << maxPseudorangeErrorMeters << '\n';
    ASSERT_GT(pseudorangeVerifiedCount, 0)
        << "Expected at least one pseudorange sample to cross-check against simulator ground truth";
    EXPECT_LT(maxPseudorangeErrorMeters, 1000.0)
        << "Pseudorange reconstruction diverges from simulator ground truth by up to " << maxPseudorangeErrorMeters
        << " m";

    EphemerisAccumulator accumulator;
    for (const GPSOpenCl::NavDecoderOutput &out : sink->navDecoderOutputs)
    {
        accumulator.ingest(out);
    }

    std::vector<int> completedPrns = accumulator.completedPrns();

    std::cerr << "DEBUG completedPrns.size()=" << completedPrns.size() << " prns=[";
    for (int prn : completedPrns) std::cerr << prn << " ";
    std::cerr << "]" << std::endl;
    std::cerr << "DEBUG navDecoderOutputs.size()=" << sink->navDecoderOutputs.size() << std::endl;
    std::map<int, int> subframeCountByPrn;
    std::map<int, uint8_t> subframeMaskByPrn;
    for (const GPSOpenCl::NavDecoderOutput &out : sink->navDecoderOutputs)
    {
        if (!out.isValid) continue;
        subframeCountByPrn[out.svId]++;
        if (out.subframeId >= 1 && out.subframeId <= 3)
        {
            subframeMaskByPrn[out.svId] |= static_cast<uint8_t>(1u << (out.subframeId - 1));
        }
    }
    for (const auto &entry : subframeCountByPrn)
    {
        int mask = subframeMaskByPrn.count(entry.first) ? subframeMaskByPrn[entry.first] : 0;
        std::cerr << "DEBUG PRN " << entry.first << " valid subframe decodes=" << entry.second << " subframe123Mask=0b"
                  << ((mask >> 2) & 1) << ((mask >> 1) & 1) << (mask & 1) << std::endl;
    }

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

        EXPECT_NEAR(decoded.sqrtA, truthEphem.sqrtA, 1e-4) << "PRN " << prn << " sqrtA";
        EXPECT_NEAR(decoded.e, truthEphem.e, 1e-8) << "PRN " << prn << " e";
        EXPECT_NEAR(decoded.M0, truthEphem.M0, 1e-7) << "PRN " << prn << " M0";
        EXPECT_NEAR(decoded.deltaN, truthEphem.deltaN, 1e-11) << "PRN " << prn << " deltaN";
        EXPECT_NEAR(decoded.toe, truthEphem.toe, 20.0) << "PRN " << prn << " toe";
        EXPECT_NEAR(decoded.i0, truthEphem.i0, 1e-7) << "PRN " << prn << " i0";
        EXPECT_NEAR(decoded.idot, truthEphem.idot, 1e-11) << "PRN " << prn << " idot";
        EXPECT_NEAR(decoded.omega0, truthEphem.omega0, 1e-7) << "PRN " << prn << " omega0";
        EXPECT_NEAR(decoded.omega, truthEphem.omega, 1e-7) << "PRN " << prn << " omega";
        EXPECT_NEAR(decoded.omegaDot, truthEphem.omegaDot, 1e-11) << "PRN " << prn << " omegaDot";
        EXPECT_NEAR(decoded.Cuc, truthEphem.Cuc, 1e-7) << "PRN " << prn << " Cuc";
        EXPECT_NEAR(decoded.Cus, truthEphem.Cus, 1e-7) << "PRN " << prn << " Cus";
        EXPECT_NEAR(decoded.Crc, truthEphem.Crc, 0.05) << "PRN " << prn << " Crc";
        EXPECT_NEAR(decoded.Crs, truthEphem.Crs, 0.05) << "PRN " << prn << " Crs";
        EXPECT_NEAR(decoded.Cic, truthEphem.Cic, 1e-7) << "PRN " << prn << " Cic";
        EXPECT_NEAR(decoded.Cis, truthEphem.Cis, 1e-7) << "PRN " << prn << " Cis";
        EXPECT_NEAR(decoded.af0, truthEphem.af0, 1e-9) << "PRN " << prn << " af0";
        EXPECT_NEAR(decoded.af1, truthEphem.af1, 1e-12) << "PRN " << prn << " af1";
        EXPECT_NEAR(decoded.af2, truthEphem.af2, 1e-15) << "PRN " << prn << " af2";
        EXPECT_NEAR(decoded.tgd, truthEphem.tgd, 1e-9) << "PRN " << prn << " tgd";
        // Subframe 1's broadcast WN field is a raw 10-bit (mod-1024) value per ICD-GPS-200, while RINEX
        // stores the un-rolled continuous GPS week - reduce the truth value the same way before comparing.
        EXPECT_EQ(decoded.weekNumber, truthEphem.weekNumber % 1024) << "PRN " << prn;

        verifiedCount++;
    }
    EXPECT_GT(verifiedCount, 0) << "No completed ephemeris had a matching RINEX record to verify against";

    ASSERT_FALSE(sink->pvtSolverOutputs.empty()) << "Expected at least one PVT solution attempt";

    const double trueEcefX = truth.front().trueReceiverPosXEcefM;
    const double trueEcefY = truth.front().trueReceiverPosYEcefM;
    const double trueEcefZ = truth.front().trueReceiverPosZEcefM;

    // A single-epoch WLS receiver has occasional transient outlier fixes (a faulted satellite is
    // only excluded once its residual or anchor age exposes it), so the fix quality is judged on
    // robust statistics plus a hard cap that would still catch any millisecond-scale timing
    // regression.
    std::vector<double> fixDistances;
    for (const GPSOpenCl::PvtSolverOutput &pvt : sink->pvtSolverOutputs)
    {
        if (!pvt.isValid)
        {
            continue;
        }

        double dx = pvt.ecefXMeters - trueEcefX;
        double dy = pvt.ecefYMeters - trueEcefY;
        double dz = pvt.ecefZMeters - trueEcefZ;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        std::cerr << "DEBUG pvtfix distance=" << distance << '\n';

        EXPECT_LT(distance, 500.0) << "PVT solution " << distance << " m from true receiver position";
        fixDistances.push_back(distance);
    }
    EXPECT_FALSE(fixDistances.empty()) << "Expected at least one valid PVT solution within " << blocksAvailable
                                       << " blocks";

    if (!fixDistances.empty())
    {
        std::sort(fixDistances.begin(), fixDistances.end());
        const double median = fixDistances[fixDistances.size() / 2];
        const double p95 = fixDistances[(fixDistances.size() * 95) / 100];
        std::cerr << "DEBUG pvtfix stats count=" << fixDistances.size() << " median=" << median << " p95=" << p95
                  << '\n';
        EXPECT_LT(median, 15.0) << "Median PVT position error " << median << " m";
        EXPECT_LT(p95, 50.0) << "95th-percentile PVT position error " << p95 << " m";
    }

    std::remove(iqFile.c_str());
}

TEST(GroundTruthVerificationTest, SubframeStartCodePhaseMatchesSimulatorTruth)
{
    if (std::getenv("GPSOPENCL_RUN_INTEGRATION_TESTS") == nullptr)
    {
        GTEST_SKIP() << "Slow subframe-timing verification skipped by default. "
                     << "Set GPSOPENCL_RUN_INTEGRATION_TESTS=1 to run it.";
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

    std::string iqFile = "ground_truth_iq_bitsync.bin";
    std::string truthFile = "ground_truth_records_bitsync.bin";
    std::string cmd = gpsSimBin + " -e " + navFile + " -l 48.1173,11.5167,545.4 -s 4096000 -b 8 -d 40 -o " + iqFile +
        " -G " + truthFile + " > /dev/null 2>&1";
    int sysRet = std::system(cmd.c_str());
    ASSERT_EQ(sysRet, 0);

    std::vector<GroundTruthRecord> truth;
    ASSERT_TRUE(readGroundTruth(truthFile, &truth));
    ASSERT_FALSE(truth.empty());

    std::map<int, std::vector<GroundTruthRecord>> truthByPrn;
    for (const GroundTruthRecord &record : truth)
    {
        truthByPrn[record.prn].push_back(record);
    }

    GPSOpenCl::ComplexFloatVector inputSignal;
    TestUtils::readFromFileBinaryIQ8(iqFile.c_str(), &inputSignal);
    ASSERT_GE(inputSignal.size(), 4096u);

    GPSOpenCl::Settings settings;
    settings.captureSettings();

    GPSOpenCl::Application app(settings.configuration);
    auto sink = std::make_shared<CapturingSink>();
    app.setSink(sink);

    int codeLength = settings.configuration.acquisitionInput.numberOfSamplesPerCode;
    ASSERT_GT(codeLength, 0);

    int blocksAvailable = static_cast<int>(inputSignal.size() / static_cast<size_t>(codeLength));

    std::map<int, size_t> seenSubframeStartSampleByPrn;
    int checkedCount = 0;

    for (int blockIdx = 0; blockIdx < blocksAvailable; blockIdx++)
    {
        auto start = inputSignal.begin() + static_cast<long>(blockIdx) * codeLength;
        auto end = start + codeLength;
        GPSOpenCl::ComplexFloatVector block(start, end);
        app.processBlock(block, static_cast<uint32_t>(blockIdx));

        for (const GPSOpenCl::Application::ChannelDiagnostic &diag : app.getChannelDiagnostics())
        {
            auto seenIt = seenSubframeStartSampleByPrn.find(diag.svId);
            if (seenIt != seenSubframeStartSampleByPrn.end() && seenIt->second == diag.subframeStartSample)
            {
                continue;
            }
            seenSubframeStartSampleByPrn[diag.svId] = diag.subframeStartSample;

            auto prnTruth = truthByPrn.find(diag.svId);
            if (prnTruth == truthByPrn.end())
            {
                continue;
            }

            auto findClosest = [&](double queryTow) -> const GroundTruthRecord *
            {
                double bestDt = 1e9;
                const GroundTruthRecord *best = findClosestRecord(prnTruth->second, queryTow, &bestDt);
                return (best != nullptr && bestDt <= 0.1) ? best : nullptr;
            };

            const GroundTruthRecord *closestAtStart = findClosest(diag.subframeStartTow);
            if (closestAtStart != nullptr)
            {
                double chipDiff =
                    static_cast<double>(diag.codePhaseAtSubframeStart) - closestAtStart->trueCodePhaseChips;
                chipDiff = std::fmod(chipDiff + 1534.5, 1023.0) - 511.5;

                std::cerr << "DEBUG bitsync PRN " << diag.svId << " bitSyncPhase=" << diag.bitSyncPhase
                          << " subframeStartTow=" << diag.subframeStartTow
                          << " subframeStartSample=" << diag.subframeStartSample
                          << " codePhaseAtSubframeStart=" << diag.codePhaseAtSubframeStart
                          << " truthCodePhase=" << closestAtStart->trueCodePhaseChips << " chipDiffWrapped=" << chipDiff
                          << " metersEquivalent=" << (chipDiff / GPSOpenCl::GPS_CA_CODE_FREQUENCY_HZ) * 299792458.0
                          << '\n';

                EXPECT_LT(std::fabs(chipDiff), 5.0)
                    << "PRN " << diag.svId << " code phase at subframe start diverges from truth by " << chipDiff
                    << " chips (bitSyncPhase=" << diag.bitSyncPhase << ")";
                checkedCount++;
            }

            const GroundTruthRecord *closestNow = findClosest(diag.candidateNowTow);
            if (closestNow != nullptr)
            {
                double chipDiffNow = static_cast<double>(diag.codePhaseNow) - closestNow->trueCodePhaseChips;
                chipDiffNow = std::fmod(chipDiffNow + 1534.5, 1023.0) - 511.5;

                std::cerr << "DEBUG elapsedcheck PRN " << diag.svId << " elapsedSeconds=" << diag.elapsedSeconds
                          << " candidateNowTow=" << diag.candidateNowTow << " codePhaseNow=" << diag.codePhaseNow
                          << " truthCodePhase=" << closestNow->trueCodePhaseChips << " chipDiffWrapped=" << chipDiffNow
                          << " metersEquivalent=" << (chipDiffNow / GPSOpenCl::GPS_CA_CODE_FREQUENCY_HZ) * 299792458.0
                          << '\n';

                EXPECT_LT(std::fabs(chipDiffNow), 5.0)
                    << "PRN " << diag.svId << " code phase 'now' (elapsedSeconds=" << diag.elapsedSeconds
                    << ") diverges from truth by " << chipDiffNow << " chips";
                checkedCount++;
            }
        }
    }

    std::cerr << "DEBUG bitsync checkedCount=" << checkedCount << '\n';
    EXPECT_GT(checkedCount, 0) << "Expected at least one subframe-start code-phase sample to check against truth";

    std::remove(iqFile.c_str());
    std::remove(truthFile.c_str());
}
}
