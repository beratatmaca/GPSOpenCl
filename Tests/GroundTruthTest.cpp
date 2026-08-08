#include "Acquisition/GPSOpenClAcquisition.hpp"
#include "Acquisition/GPSOpenClCaCodeGenerator.hpp"
#include "Application/GPSOpenClApplication.hpp"
#include "Common/GPSOpenClCommon.hpp"
#include "Common/GPSOpenClSettings.hpp"
#include "Common/GPSOpenClStructs.hpp"
#include "Gpu/GPSOpenClSpectrumEngine.hpp"
#include "GroundTruthRecord.hpp"
#include "GroundTruthTestUtils.hpp"
#include "NavDecode/GPSOpenClNavigationDecoder.hpp"
#include "RinexNavParser.hpp"
#include "TestUtils.hpp"
#include "Tracking/GPSOpenClChannel.hpp"

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
#include <random>
#include <sstream>
#include <vector>

namespace GPSOpenClTest
{
namespace
{
constexpr unsigned int DEFAULT_SEED = 424'242u;

// Resolves the base random seed: GPSOPENCL_TEST_SEED overrides the fixed default, so CI stays
// reproducible while a flake can still be fuzzed/reproduced locally by setting the env var.
unsigned int resolveBaseSeed()
{
    if (const char *envSeed = std::getenv("GPSOPENCL_TEST_SEED"))
    {
        return static_cast<unsigned int>(std::strtoul(envSeed, nullptr, 10));
    }
    return DEFAULT_SEED;
}

// Each randomized input gets its own stream (base seed + a fixed per-purpose salt) so the two test
// cases don't accidentally draw identical scenarios, while everything stays reproducible from one seed.
std::mt19937 makeRng(unsigned int streamSalt, const std::string &label)
{
    const unsigned int baseSeed = resolveBaseSeed();
    const unsigned int seed = baseSeed + streamSalt;
    std::cerr << "GroundTruthTest[" << label << "] seed=" << seed << " (rerun with GPSOPENCL_TEST_SEED=" << baseSeed << " to reproduce)\n";
    return std::mt19937(seed);
}

struct RandomPosition
{
    double latDeg;
    double lonDeg;
    double altM;
};

// Avoids the poles (weaker/more degenerate satellite geometry) and picks a modest altitude band;
// anywhere in this range gives a normal, well-conditioned GPS constellation view.
RandomPosition randomPosition(std::mt19937 &rng)
{
    std::uniform_real_distribution<double> latDist(-60.0, 60.0);
    std::uniform_real_distribution<double> lonDist(-180.0, 180.0);
    std::uniform_real_distribution<double> altDist(0.0, 1000.0);
    return {latDist(rng), lonDist(rng), altDist(rng)};
}

std::string formatPositionArg(const RandomPosition &pos)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(7) << "-l " << pos.latDeg << "," << pos.lonDeg << "," << pos.altM;
    return oss.str();
}

struct Ecef
{
    double x;
    double y;
    double z;
};

// Standard WGS84 geodetic-to-ECEF transform (matches Tools/gps-sdr-sim's own llh2xyz()).
Ecef geodeticToEcef(double latDeg, double lonDeg, double altM)
{
    const double a = GPSOpenCl::WGS84_SEMI_MAJOR_AXIS_M;
    const double f = 1.0 / 298.257223563;
    const double e2 = f * (2.0 - f);
    const double lat = latDeg * M_PI / 180.0;
    const double lon = lonDeg * M_PI / 180.0;
    const double sinLat = std::sin(lat);
    const double cosLat = std::cos(lat);
    const double primeVerticalRadius = a / std::sqrt(1.0 - e2 * sinLat * sinLat);
    const double nph = primeVerticalRadius + altM;
    return {nph * cosLat * std::cos(lon), nph * cosLat * std::sin(lon), (((1.0 - e2) * primeVerticalRadius) + altM) * sinLat};
}

// Writes a gps-sdr-sim "-u" user-motion file (ECEF x,y,z at 0.1s steps) tracing a horizontal circle
// of random radius/period around a random center, so the dynamic scenario exercises a different
// receiver trajectory every seed instead of always the checked-in circle.csv.
void writeRandomCircleMotionFile(std::mt19937 &rng, const std::string &path)
{
    const RandomPosition center = randomPosition(rng);
    std::uniform_real_distribution<double> radiusDist(50.0, 300.0);
    std::uniform_real_distribution<double> periodDist(40.0, 120.0);
    const double radius = radiusDist(rng);
    const double period = periodDist(rng);

    const double latRad = center.latDeg * M_PI / 180.0;
    const double lonRad = center.lonDeg * M_PI / 180.0;
    const double eastX = -std::sin(lonRad);
    const double eastY = std::cos(lonRad);
    const double northX = -std::sin(latRad) * std::cos(lonRad);
    const double northY = -std::sin(latRad) * std::sin(lonRad);
    const double northZ = std::cos(latRad);

    const Ecef centerEcef = geodeticToEcef(center.latDeg, center.lonDeg, center.altM);

    std::cerr << "GroundTruthTest circle: center=(" << center.latDeg << "," << center.lonDeg << "," << center.altM << ") radiusM=" << radius << " periodS=" << period << '\n';

    std::ofstream out(path);
    constexpr double STEP_SECONDS = 0.1;
    constexpr int NUM_STEPS = 600;    // 60 s of waypoints, well beyond what any scenario duration needs.
    for (int i = 0; i < NUM_STEPS; i++)
    {
        const double t = i * STEP_SECONDS;
        const double angle = 2.0 * M_PI * t / period;
        const double east = radius * std::cos(angle);
        const double north = radius * std::sin(angle);
        const double x = centerEcef.x + (east * eastX) + (north * northX);
        const double y = centerEcef.y + (east * eastY) + (north * northY);
        const double z = centerEcef.z + (north * northZ);
        out << std::fixed << std::setprecision(3) << t << "," << x << "," << y << "," << z << "\n";
    }
}

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

const GroundTruthRecord *findClosestRecord(const std::vector<GroundTruthRecord> &records, double targetTimeSec, double *outDt)
{
    auto lowerBound = std::lower_bound(records.begin(), records.end(), targetTimeSec, [](const GroundTruthRecord &record, double t) { return record.gpsTimeSec < t; });

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

// Returns false (and asserts nothing) when the RINEX file has no record for prn, so the
// multi-channel caller can skip it; the single-channel caller asserts the true result instead.
bool ephemerisMatchesRinex(int prn, const std::string &navFile, const GPSOpenCl::GpsEphemeris &decoded)
{
    GPSOpenCl::GpsEphemeris truthEphem{};
    if (!RinexNavParser::findEphemeris(navFile, prn, &truthEphem))
    {
        return false;
    }

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
    return true;
}

/** @brief Run a short (2s) multi-satellite scenario through the full Application pipeline and
 * check acquisition and tracking Doppler against simulator ground truth. Shared by the static and
 * moving-receiver scenarios; scenarioArgs is the gps-sdr-sim position ("-l lat,lon,alt") or motion
 * ("-u file.csv") flag that distinguishes them. */
void runMultiChannelSmokeScenario(const std::string &gpsSimBin, const std::string &navFile, const std::string &scenarioArgs, const std::string &scenarioName)
{
    const std::string iqFile = "ground_truth_" + scenarioName + "_iq.bin";
    const std::string truthFile = "ground_truth_" + scenarioName + "_truth.bin";

    const std::string cmd = gpsSimBin + " -e " + navFile + " " + scenarioArgs + " -s 4096000 -b 8 -d 2 -o " + iqFile + " -G " + truthFile + " > /dev/null 2>&1";
    ASSERT_EQ(std::system(cmd.c_str()), 0) << scenarioName << " scenario: gps-sdr-sim invocation failed";

    std::vector<GroundTruthRecord> truth;
    ASSERT_TRUE(readGroundTruth(truthFile, &truth)) << scenarioName << " scenario";
    ASSERT_FALSE(truth.empty()) << scenarioName << " scenario";
    ASSERT_EQ(truth.front().structVersion, 2u) << scenarioName << " scenario";

    GPSOpenCl::ComplexFloatVector inputSignal;
    TestUtils::readFromFileBinaryIQ8(iqFile.c_str(), &inputSignal);
    ASSERT_GE(inputSignal.size(), 4096u) << scenarioName << " scenario";

    GPSOpenCl::Settings settings;
    settings.captureSettings();

    GPSOpenCl::Application app(settings.configuration);
    auto sink = std::make_shared<CapturingSink>();
    app.setSink(sink);

    const int codeLength = settings.configuration.acquisitionInput.numberOfSamplesPerCode;
    ASSERT_GT(codeLength, 0);

    const int blocksAvailable = static_cast<int>(inputSignal.size() / static_cast<size_t>(codeLength));
    const int blocksToProcess = std::min(blocksAvailable, 450);
    ASSERT_GE(blocksToProcess, 2) << scenarioName << " scenario";

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

    ASSERT_FALSE(sink->acquisitionOutputs.empty()) << scenarioName << " scenario";

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
        ASSERT_NE(it, firstTruthByPrn.end()) << scenarioName << " scenario: acquired PRN " << acq.prn << " has no ground truth record";

        const double dopplerDiff = std::fabs(acq.peakFrequencyHz - it->second.trueDopplerHz);
        EXPECT_LT(dopplerDiff, 400.0) << scenarioName << " scenario: PRN " << acq.prn << " acquired Doppler " << acq.peakFrequencyHz << " Hz too far from true Doppler "
                                      << it->second.trueDopplerHz << " Hz";
        acquiredCount++;
    }
    EXPECT_GE(acquiredCount, 10) << scenarioName << " scenario: expected most of the scenario's real satellites to acquire";

    ASSERT_FALSE(sink->trackingOutputs.empty()) << scenarioName << " scenario";

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

        const double dopplerDiff = std::fabs(entry.second.carrierFreqHz - it->second.trueDopplerHz);
        EXPECT_LT(dopplerDiff, 50.0) << scenarioName << " scenario: PRN " << entry.first << " tracked Doppler " << entry.second.carrierFreqHz << " Hz too far from true Doppler "
                                     << it->second.trueDopplerHz << " Hz";
        convergedCount++;
    }
    EXPECT_GT(convergedCount, 0) << scenarioName << " scenario: expected at least one channel to reach confirmed Tracking";

    std::remove(iqFile.c_str());
    std::remove(truthFile.c_str());
}

/** @brief Run one long (180s) static multi-satellite scenario and check pseudorange, decoded
 * ephemeris, PVT fix position, and subframe-start code phase together against simulator ground
 * truth in a single gps-sdr-sim invocation. Gated behind GPSOPENCL_RUN_INTEGRATION_TESTS since a
 * real ephemeris decode needs several 30s subframe cycles.
 *
 * Subframes cycle 1-2-3-4-5 every 30s on a fixed GPS-time schedule shared by every satellite in
 * the scenario. Since the scenario's start time is frame-aligned, subframe 1's preamble falls at
 * the very first instant, before acquisition/confirm can lock on to catch it - every satellite
 * misses it in the first cycle and only catches it ~30s later in the second. PVT additionally
 * needs 4 satellites with complete ephemeris simultaneously; the weaker ones (~15 dB-Hz below the
 * strongest) need multiple clean cycles to get every word's parity right. 180s (six full cycles)
 * gives them a realistic chance; empirically 45-90s only ever got the single strongest satellite.
 *
 * Pseudorange comparisons start one solve interval after the first valid PVT fix. Before the
 * first fix the receiver has no validated position, so millisecond snapping is disabled
 * (Application::isReferencePositionTrusted) and satellites carrying the inherent one-code-period
 * bit-edge ambiguity legitimately produce ~300 km pseudorange outliers that only the solver's
 * residual gate handles. Judging raw pseudoranges there would test a guarantee the receiver
 * never makes. The extra solve interval skips the transition solve, whose samples were assembled
 * before the fix existed. This gating replaced an earlier known issue: transmit times used to be
 * snapped against the unvalidated compiled-in seed position for the first ten solves, corrupting
 * them by multiple whole code periods whenever the true position was far from the seed - invisible
 * for as long as every ground-truth test used the seed location itself as its receiver position. */
void runMultiChannelDeepVerification(const std::string &gpsSimBin, const std::string &navFile, const RandomPosition &position)
{
    const std::string iqFile = "ground_truth_deep_iq.bin";
    const std::string truthFile = "ground_truth_deep_truth.bin";
    const std::string cmd = gpsSimBin + " -e " + navFile + " " + formatPositionArg(position) + " -s 4096000 -b 8 -d 180 -o " + iqFile + " -G " + truthFile + " > /dev/null 2>&1";
    ASSERT_EQ(std::system(cmd.c_str()), 0);

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

    const int codeLength = settings.configuration.acquisitionInput.numberOfSamplesPerCode;
    ASSERT_GT(codeLength, 0);

    const int blocksAvailable = static_cast<int>(inputSignal.size() / static_cast<size_t>(codeLength));

    const double speedOfLight = 299792458.0;
    std::map<int, double> maxErrorByPrn;
    std::map<int, double> sumErrorByPrn;
    std::map<int, int> countByPrn;

    std::map<int, size_t> seenSubframeStartSampleByPrn;
    int codePhaseCheckedCount = 0;

    // Both the measured pseudorange and the simulator's truePseudorangeM are referenced to the
    // satellite's own clock, so they compare directly with no clock correction. The receiver's
    // synthesized common epoch legitimately carries an arbitrary offset (absorbed by the clock-bias
    // state in the solver), so errors are judged relative to the per-epoch mean across satellites.
    int firstValidFixBlock = -1;
    const int solveIntervalBlocks = settings.configuration.pvtSolverInput.fixOutputIntervalBlocks;

    std::vector<std::pair<int, double>> epochDiffs;
    for (int blockIdx = 0; blockIdx < blocksAvailable; blockIdx++)
    {
        auto start = inputSignal.begin() + static_cast<long>(blockIdx) * codeLength;
        auto end = start + codeLength;
        GPSOpenCl::ComplexFloatVector block(start, end);
        app.processBlock(block, static_cast<uint32_t>(blockIdx));

        if (firstValidFixBlock < 0 && !sink->pvtSolverOutputs.empty() && sink->pvtSolverOutputs.back().isValid)
        {
            firstValidFixBlock = blockIdx;
        }

        // Pseudoranges are only judged from one solve interval after the first valid fix onward
        // (see the function comment); until then the samples reflect unrepaired bit-edge ambiguity.
        const bool pseudorangeJudgingActive = firstValidFixBlock >= 0 && blockIdx >= firstValidFixBlock + solveIntervalBlocks;

        epochDiffs.clear();
        for (const GPSOpenCl::Application::PseudorangeSample &sample : app.getLastPseudorangeSamples())
        {
            if (!pseudorangeJudgingActive)
            {
                break;
            }

            auto prnTruth = truthByPrn.find(sample.svId);
            if (prnTruth == truthByPrn.end())
            {
                continue;
            }

            const double receiverTimeSec = sample.transmitTimeSeconds + (sample.measuredPseudorangeMeters / speedOfLight);
            double closestDt = 1e9;
            const GroundTruthRecord *closest = findClosestRecord(prnTruth->second, receiverTimeSec, &closestDt);

            // Below the elevation mask, the solver never trusts this satellite for a real fix either
            // (PVTSolver::solvePosition drops it once a fix exists) - hold the raw measurement to the
            // same standard production actually uses, rather than a stricter one it never has to meet.
            if (closest != nullptr && closestDt < 0.002 && closest->trueElevationDeg >= settings.configuration.pvtSolverInput.elevationMaskDeg)
            {
                epochDiffs.emplace_back(sample.svId, sample.measuredPseudorangeMeters - closest->truePseudorangeM);
            }
        }

        if (epochDiffs.size() >= 2)
        {
            double meanDiff = 0.0;
            for (const auto &entry : epochDiffs)
            {
                meanDiff += entry.second;
            }
            meanDiff /= static_cast<double>(epochDiffs.size());

            for (const auto &entry : epochDiffs)
            {
                const double err = std::fabs(entry.second - meanDiff);
                maxErrorByPrn[entry.first] = std::max(maxErrorByPrn[entry.first], err);
                sumErrorByPrn[entry.first] += err;
                countByPrn[entry.first]++;
            }
        }

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

            if (const GroundTruthRecord *closestAtStart = findClosest(diag.subframeStartTow))
            {
                double chipDiff = static_cast<double>(diag.codePhaseAtSubframeStart) - closestAtStart->trueCodePhaseChips;
                chipDiff = std::fmod(chipDiff + 1534.5, 1023.0) - 511.5;
                EXPECT_LT(std::fabs(chipDiff), 5.0) << "PRN " << diag.svId << " code phase at subframe start diverges from truth by " << chipDiff
                                                    << " chips (bitSyncPhase=" << diag.bitSyncPhase << ")";
                codePhaseCheckedCount++;
            }

            if (const GroundTruthRecord *closestNow = findClosest(diag.candidateNowTow))
            {
                double chipDiffNow = static_cast<double>(diag.codePhaseNow) - closestNow->trueCodePhaseChips;
                chipDiffNow = std::fmod(chipDiffNow + 1534.5, 1023.0) - 511.5;
                EXPECT_LT(std::fabs(chipDiffNow), 5.0) << "PRN " << diag.svId << " code phase 'now' (elapsedSeconds=" << diag.elapsedSeconds << ") diverges from truth by "
                                                       << chipDiffNow << " chips";
                codePhaseCheckedCount++;
            }
        }
    }

    int pseudorangeVerifiedCount = 0;
    double maxPseudorangeErrorMeters = 0.0;
    for (const auto &entry : countByPrn)
    {
        const int prn = entry.first;
        pseudorangeVerifiedCount += entry.second;
        maxPseudorangeErrorMeters = std::max(maxPseudorangeErrorMeters, maxErrorByPrn[prn]);
        std::cerr << "GroundTruthTest pseudorange PRN " << prn << " count=" << entry.second << " meanErr=" << (sumErrorByPrn[prn] / entry.second)
                  << " maxErr=" << maxErrorByPrn[prn] << '\n';
    }
    ASSERT_GT(pseudorangeVerifiedCount, 0) << "Expected at least one pseudorange sample to cross-check against simulator ground truth";
    EXPECT_LT(maxPseudorangeErrorMeters, 1000.0) << "Pseudorange reconstruction diverges from simulator ground truth by up to " << maxPseudorangeErrorMeters << " m";

    EXPECT_GT(codePhaseCheckedCount, 0) << "Expected at least one subframe-start code-phase sample to check against truth";

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
        if (ephemerisMatchesRinex(prn, navFile, accumulator.get(prn)))
        {
            verifiedCount++;
        }
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

        const double dx = pvt.ecefXMeters - trueEcefX;
        const double dy = pvt.ecefYMeters - trueEcefY;
        const double dz = pvt.ecefZMeters - trueEcefZ;
        const double distance = std::sqrt((dx * dx) + (dy * dy) + (dz * dz));

        EXPECT_LT(distance, 500.0) << "PVT solution " << distance << " m from true receiver position";
        fixDistances.push_back(distance);
    }
    EXPECT_FALSE(fixDistances.empty()) << "Expected at least one valid PVT solution within " << blocksAvailable << " blocks";

    if (!fixDistances.empty())
    {
        std::sort(fixDistances.begin(), fixDistances.end());
        const double median = fixDistances[fixDistances.size() / 2];
        const double p95 = fixDistances[(fixDistances.size() * 95) / 100];
        EXPECT_LT(median, 15.0) << "Median PVT position error " << median << " m";
        EXPECT_LT(p95, 50.0) << "95th-percentile PVT position error " << p95 << " m";
    }

    std::remove(iqFile.c_str());
    std::remove(truthFile.c_str());
}

bool gpsSimAvailable(const std::string &gpsSimBin)
{
    std::ifstream checkSim(gpsSimBin);
    return checkSim.is_open();
}
}    // namespace

// Isolates single-satellite acquisition, tracking, and ephemeris decode by driving Acquisition and
// Channel directly (bypassing Application/multi-channel scheduling), against a randomized receiver
// position. Set GPSOPENCL_RUN_INTEGRATION_TESTS=1 to additionally verify the decoded ephemeris
// against RINEX (needs a longer, ~60s capture to complete subframes 1-3).
TEST(GroundTruthTest, SingleChannelMatchesSimulatorGroundTruth)
{
    const std::string gpsSimBin = "../Tools/gps-sdr-sim/gps-sdr-sim";
    const std::string navFile = "Tools/gps-sdr-sim/brdc0010.22n";
    if (!gpsSimAvailable(gpsSimBin))
    {
        GTEST_SKIP() << "gps-sdr-sim binary not found at " << gpsSimBin << ". Skipping single-channel ground-truth test.";
        return;
    }

    std::mt19937 rng = makeRng(1, "SingleChannel");
    const RandomPosition position = randomPosition(rng);

    const std::string iqFile = "single_channel_iq.bin";
    const std::string truthFile = "single_channel_truth.bin";
    const std::string cmd = gpsSimBin + " -e " + navFile + " " + formatPositionArg(position) + " -s 4096000 -b 8 -d 2 -o " + iqFile + " -G " + truthFile + " > /dev/null 2>&1";
    ASSERT_EQ(std::system(cmd.c_str()), 0);

    std::vector<GroundTruthRecord> truth;
    ASSERT_TRUE(readGroundTruth(truthFile, &truth));
    ASSERT_FALSE(truth.empty());

    const int prn = strongestPrn(truth);
    ASSERT_GT(prn, 0) << "No satellite found in ground truth";
    const GroundTruthRecord firstTruth = firstRecordForPrn(truth, prn);

    GPSOpenCl::ComplexFloatVector inputSignal;
    TestUtils::readFromFileBinaryIQ8(iqFile.c_str(), &inputSignal);
    ASSERT_GE(inputSignal.size(), 4096u);

    GPSOpenCl::Settings settings;
    settings.captureSettings();

    GPSOpenCl::SpectrumEngine gpu;
    GPSOpenCl::CaCodeGenerator code(settings.configuration);
    code.createLookupTable(&gpu);

    GPSOpenCl::Acquisition acquisition(settings.configuration);
    GPSOpenCl::Channel channel;
    channel.svId = prn;

    auto sink = std::make_shared<CapturingSink>();
    channel.setSink(sink);

    const int codeLength = settings.configuration.acquisitionInput.numberOfSamplesPerCode;
    ASSERT_GT(codeLength, 0);
    ASSERT_GE(inputSignal.size(), static_cast<size_t>(codeLength));

    GPSOpenCl::ComplexFloatVector firstBlock(inputSignal.begin(), inputSignal.begin() + codeLength);
    acquisition.correlate(firstBlock, &gpu, &code, &channel);

    int peakIndex = 0;
    float peakValue = 0.0f, peakFreq = 0.0f, meanValue = 0.0f, cn0 = 0.0f, peakRatio = 0.0f;
    channel.getAcquisitionResults(&peakIndex, &peakValue, &peakFreq, &meanValue, &cn0, &peakRatio);

    const float dopplerHz = -peakFreq;
    const double dopplerDiff = std::fabs(static_cast<double>(dopplerHz) - firstTruth.trueDopplerHz);
    EXPECT_LT(dopplerDiff, 400.0) << "PRN " << prn << " acquired Doppler " << dopplerHz << " Hz too far from true Doppler " << firstTruth.trueDopplerHz << " Hz";

    const float numSamplesFloat = static_cast<float>(codeLength);
    const int reflectedPeakIndex = (codeLength - peakIndex) % codeLength;
    const float codePhaseChips = (static_cast<float>(reflectedPeakIndex) / numSamplesFloat) * GPSOpenCl::GPS_CA_CODE_LENGTH;

    channel.setAcquired(true);
    channel.initTracking(settings.configuration, dopplerHz, codePhaseChips);

    const int totalBlocks = static_cast<int>(inputSignal.size() / static_cast<size_t>(codeLength));
    const int blocksToRun = std::min(totalBlocks - 1, 450);
    ASSERT_GE(blocksToRun, 2);

    for (int b = 1; b <= blocksToRun; b++)
    {
        GPSOpenCl::ComplexFloatVector block(inputSignal.begin() + static_cast<long>(b) * codeLength, inputSignal.begin() + static_cast<long>(b + 1) * codeLength);
        channel.trackBlock(block);
    }

    EXPECT_EQ(channel.getState(), GPSOpenCl::ChannelState::Tracking) << "Expected the channel to reach confirmed Tracking within " << blocksToRun << " blocks";

    GPSOpenCl::TrackingOutput lastTracking{};
    ASSERT_TRUE(channel.getTrackingOutput(&lastTracking)) << "Expected tracking state to be available";
    const GroundTruthRecord lastTruth = lastRecordForPrn(truth, prn);
    const double trackedDopplerDiff = std::fabs(lastTracking.carrierFreqHz - lastTruth.trueDopplerHz);
    EXPECT_LT(trackedDopplerDiff, 50.0) << "PRN " << prn << " tracked Doppler " << lastTracking.carrierFreqHz << " Hz too far from true Doppler " << lastTruth.trueDopplerHz
                                        << " Hz";
    EXPECT_GT(lastTracking.carrierLockIndicator, 0.7) << "Expected carrier lock indicator near 1.0 once tracking is confirmed";

    std::remove(iqFile.c_str());
    std::remove(truthFile.c_str());

    if (std::getenv("GPSOPENCL_RUN_INTEGRATION_TESTS") == nullptr)
    {
        return;
    }

    const std::string longIqFile = "single_channel_iq_long.bin";
    const std::string longTruthFile = "single_channel_truth_long.bin";
    const std::string longCmd =
        gpsSimBin + " -e " + navFile + " " + formatPositionArg(position) + " -s 4096000 -b 8 -d 60 -o " + longIqFile + " -G " + longTruthFile + " > /dev/null 2>&1";
    ASSERT_EQ(std::system(longCmd.c_str()), 0);

    std::vector<GroundTruthRecord> longTruth;
    ASSERT_TRUE(readGroundTruth(longTruthFile, &longTruth));
    ASSERT_FALSE(longTruth.empty());

    GPSOpenCl::ComplexFloatVector longSignal;
    TestUtils::readFromFileBinaryIQ8(longIqFile.c_str(), &longSignal);
    ASSERT_GE(longSignal.size(), 4096u);

    GPSOpenCl::SpectrumEngine longGpu;
    GPSOpenCl::CaCodeGenerator longCode(settings.configuration);
    longCode.createLookupTable(&longGpu);

    GPSOpenCl::Acquisition longAcquisition(settings.configuration);
    GPSOpenCl::Channel longChannel;
    longChannel.svId = prn;

    ASSERT_GE(longSignal.size(), static_cast<size_t>(codeLength));
    GPSOpenCl::ComplexFloatVector longFirstBlock(longSignal.begin(), longSignal.begin() + codeLength);
    longAcquisition.correlate(longFirstBlock, &longGpu, &longCode, &longChannel);

    int longPeakIndex = 0;
    float longPeakValue = 0.0f, longPeakFreq = 0.0f, longMeanValue = 0.0f, longCn0 = 0.0f, longPeakRatio = 0.0f;
    longChannel.getAcquisitionResults(&longPeakIndex, &longPeakValue, &longPeakFreq, &longMeanValue, &longCn0, &longPeakRatio);
    const float longDopplerHz = -longPeakFreq;

    const int longReflectedPeakIndex = (codeLength - longPeakIndex) % codeLength;
    const float longCodePhaseChips = (static_cast<float>(longReflectedPeakIndex) / numSamplesFloat) * GPSOpenCl::GPS_CA_CODE_LENGTH;

    longChannel.setAcquired(true);
    longChannel.initTracking(settings.configuration, longDopplerHz, longCodePhaseChips);

    GPSOpenCl::NavigationDecoder decoder;
    const int longTotalBlocks = static_cast<int>(longSignal.size() / static_cast<size_t>(codeLength));

    for (int b = 1; b < longTotalBlocks; b++)
    {
        GPSOpenCl::ComplexFloatVector block(longSignal.begin() + static_cast<long>(b) * codeLength, longSignal.begin() + static_cast<long>(b + 1) * codeLength);
        longChannel.trackBlock(block);

        if (longChannel.getState() == GPSOpenCl::ChannelState::Tracking)
        {
            longChannel.updateNavigation(decoder);
        }
    }

    ASSERT_TRUE(longChannel.hasCompleteEphemeris()) << "Expected PRN " << prn << " to decode a complete ephemeris (subframes 1-3) within " << longTotalBlocks << " blocks";
    ASSERT_TRUE(ephemerisMatchesRinex(prn, navFile, longChannel.getAccumulatedEphemeris())) << "No matching RINEX record for PRN " << prn;

    std::remove(longIqFile.c_str());
    std::remove(longTruthFile.c_str());
}

// Drives the full receiver pipeline (Application, multiple concurrent channels) through
// gps-sdr-sim and checks the result against the simulator's exported ground truth: a static
// randomized position and a randomized moving-receiver trajectory, each checked for
// acquisition/tracking Doppler accuracy. Set GPSOPENCL_RUN_INTEGRATION_TESTS=1 to additionally run
// the slow (~180s) pseudorange, ephemeris, PVT-fix, and code-phase verification.
TEST(GroundTruthTest, MultiChannelMatchesSimulatorGroundTruth)
{
    const std::string gpsSimBin = "../Tools/gps-sdr-sim/gps-sdr-sim";
    const std::string navFile = "Tools/gps-sdr-sim/brdc0010.22n";
    if (!gpsSimAvailable(gpsSimBin))
    {
        GTEST_SKIP() << "gps-sdr-sim binary not found at " << gpsSimBin << ". Skipping multi-channel ground-truth test.";
        return;
    }

    std::mt19937 staticRng = makeRng(2, "MultiChannelStatic");
    const RandomPosition staticPosition = randomPosition(staticRng);
    std::cerr << "GroundTruthTest static position: lat=" << staticPosition.latDeg << " lon=" << staticPosition.lonDeg << " alt=" << staticPosition.altM << '\n';
    runMultiChannelSmokeScenario(gpsSimBin, navFile, formatPositionArg(staticPosition), "static");

    std::mt19937 circleRng = makeRng(3, "MultiChannelDynamic");
    const std::string motionFile = "ground_truth_dynamic_motion.csv";
    writeRandomCircleMotionFile(circleRng, motionFile);
    runMultiChannelSmokeScenario(gpsSimBin, navFile, "-u " + motionFile, "dynamic");
    std::remove(motionFile.c_str());

    if (std::getenv("GPSOPENCL_RUN_INTEGRATION_TESTS") != nullptr)
    {
        runMultiChannelDeepVerification(gpsSimBin, navFile, staticPosition);
    }
}
}    // namespace GPSOpenClTest
