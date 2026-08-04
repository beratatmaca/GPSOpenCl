#include "GPSOpenClAtmosphericCorrections.h"
#include "GPSOpenClPVTSolver.h"

#include "gtest/gtest.h"
#include <cmath>

namespace GPSOpenClTest
{
TEST(PVTSolverTest, EcefToWgs84Equator)
{
    GPSOpenCl::EcefPosition ecef;
    ecef.x = 6378137.0;
    ecef.y = 0.0;
    ecef.z = 0.0;

    GPSOpenCl::GeodeticPosition geo = GPSOpenCl::PVTSolver::ecefToWgs84(ecef);

    EXPECT_NEAR(geo.latitude, 0.0, 1e-4);
    EXPECT_NEAR(geo.longitude, 0.0, 1e-4);
    EXPECT_NEAR(geo.altitude, 0.0, 1e-3);
}

TEST(PVTSolverTest, OrbitCalculation)
{
    GPSOpenCl::GpsEphemeris ephem;
    ephem.svId = 1;
    ephem.toe = 1000.0;
    ephem.toc = 1000.0;
    ephem.sqrtA = 5153.6;
    ephem.e = 0.01;
    ephem.M0 = 0.0;
    ephem.deltaN = 0.0;
    ephem.i0 = 0.96;
    ephem.idot = 0.0;
    ephem.omega0 = 0.0;
    ephem.omegaDot = 0.0;
    ephem.omega = 0.0;
    ephem.Cuc = ephem.Cus = ephem.Crc = ephem.Crs = ephem.Cic = ephem.Cis = 0.0;
    ephem.af0 = ephem.af1 = ephem.af2 = 0.0;

    GPSOpenCl::SatelliteOrbit orbit = GPSOpenCl::PVTSolver::computeSatelliteOrbit(ephem, 1000.0);

    double radius = std::sqrt(orbit.position.x * orbit.position.x + orbit.position.y * orbit.position.y +
                              orbit.position.z * orbit.position.z);

    double expectedRadius = ephem.sqrtA * ephem.sqrtA * (1.0 - ephem.e);
    EXPECT_NEAR(radius, expectedRadius, 10.0);
}

TEST(PVTSolverTest, StructOverloadResetsOutputOnFailure)
{
    std::vector<GPSOpenCl::NavDecoderOutput> outputs(2);
    std::vector<double> measuredRanges(2, 20000000.0);
    std::vector<double> transmitTimes(2, 0.0);

    GPSOpenCl::PVTSolver solver;
    GPSOpenCl::PvtSolverOutput outputSolution{};
    outputSolution.isValid = 1;
    outputSolution.latitude = 12.34;

    bool success = solver.solvePosition(outputs, measuredRanges, transmitTimes, outputSolution);

    EXPECT_FALSE(success);
    EXPECT_EQ(outputSolution.isValid, 0u);
    EXPECT_EQ(outputSolution.latitude, 0.0);
}

TEST(PVTSolverTest, MinSatellitesGateRejectsBelowThreshold)
{
    GPSOpenCl::PvtSolverInput input{};
    input.structVersion = GPSOpenCl::STRUCT_VERSION_1;
    input.minSatellites = 6;
    input.maxPseudorangeErrMeters = 100.0;

    GPSOpenCl::PVTSolver solver(input);
    std::vector<GPSOpenCl::GpsEphemeris> ephemerides(4);
    std::vector<double> measuredRanges(4, 20000000.0);
    std::vector<double> transmitTimes(4, 0.0);
    GPSOpenCl::ReceiverPvtSolution solution;

    bool success = solver.solvePosition(ephemerides, measuredRanges, transmitTimes, solution);

    EXPECT_FALSE(success);
    EXPECT_FALSE(solution.isValid);
}

TEST(PVTSolverTest, MaxPseudorangeErrGateRejectsOutlierWithRedundantSatellites)
{
    GPSOpenCl::EcefPosition rxEcef;
    rxEcef.x = 4239828.0;
    rxEcef.y = 2447866.0;
    rxEcef.z = 4078438.0;

    const double c = 299792458.0;
    const double omegaE = 7.2921151467e-5;
    const int numSats = 5;

    std::vector<GPSOpenCl::GpsEphemeris> ephemerides(numSats);
    std::vector<double> measuredRanges(numSats);

    for (int i = 0; i < numSats; i++)
    {
        ephemerides[i].svId = i + 1;
        ephemerides[i].toe = 0.0;
        ephemerides[i].toc = 0.0;
        ephemerides[i].sqrtA = 5153.6;
        ephemerides[i].e = 0.001;
        ephemerides[i].M0 = i * 1.5;
        ephemerides[i].deltaN = 0.0;
        ephemerides[i].i0 = 0.95;
        ephemerides[i].idot = 0.0;
        ephemerides[i].omega0 = i * 1.57;
        ephemerides[i].omegaDot = 0.0;
        ephemerides[i].omega = 0.0;
        ephemerides[i].Cuc = ephemerides[i].Cus = ephemerides[i].Crc = ephemerides[i].Crs = ephemerides[i].Cic =
            ephemerides[i].Cis = 0.0;
        ephemerides[i].af0 = ephemerides[i].af1 = ephemerides[i].af2 = 0.0;

        GPSOpenCl::SatelliteOrbit orbit = GPSOpenCl::PVTSolver::computeSatelliteOrbit(ephemerides[i], 0.0);
        double dx = orbit.position.x - rxEcef.x;
        double dy = orbit.position.y - rxEcef.y;
        double dz = orbit.position.z - rxEcef.z;
        double range = std::sqrt(dx * dx + dy * dy + dz * dz);

        double travelTime = range / c;
        double sagnacX = orbit.position.x + omegaE * travelTime * orbit.position.y;
        double sagnacY = orbit.position.y - omegaE * travelTime * orbit.position.x;

        dx = sagnacX - rxEcef.x;
        dy = sagnacY - rxEcef.y;
        dz = orbit.position.z - rxEcef.z;
        measuredRanges[i] = std::sqrt(dx * dx + dy * dy + dz * dz);

        GPSOpenCl::GeodeticPosition rxGeo = GPSOpenCl::PVTSolver::ecefToWgs84(rxEcef);
        GPSOpenCl::AtmosphericInput noIonoParams{};
        GPSOpenCl::AtmosphericOutput atmo = GPSOpenCl::AtmosphericCorrections::computeCorrections(
            ephemerides[i].svId, rxGeo, rxEcef, orbit.position, 0.0, noIonoParams);
        measuredRanges[i] += atmo.ionoDelayMeters + atmo.tropoDelayMeters;
    }

    GPSOpenCl::PVTSolver solver;
    std::vector<double> transmitTimes(numSats, 0.0);

    GPSOpenCl::ReceiverPvtSolution cleanSolution;
    EXPECT_TRUE(solver.solvePosition(ephemerides, measuredRanges, transmitTimes, cleanSolution));

    measuredRanges[2] += 10000.0;
    GPSOpenCl::ReceiverPvtSolution corruptedSolution;
    bool success = solver.solvePosition(ephemerides, measuredRanges, transmitTimes, corruptedSolution);

    EXPECT_FALSE(success);
}

TEST(PVTSolverTest, IonosphericCorrectionAppliedWhenParamsSet)
{
    GPSOpenCl::EcefPosition rxEcef;
    rxEcef.x = 4239828.0;
    rxEcef.y = 2447866.0;
    rxEcef.z = 4078438.0;

    const double c = 299792458.0;
    const double omegaE = 7.2921151467e-5;

    GPSOpenCl::AtmosphericInput ionoParams{};
    ionoParams.alpha0 = 0.1118e-07;
    ionoParams.alpha1 = 0.7451e-08;
    ionoParams.alpha2 = -0.5960e-07;
    ionoParams.alpha3 = -0.1192e-06;
    ionoParams.beta0 = 0.1167e+06;
    ionoParams.beta1 = -0.2294e+05;
    ionoParams.beta2 = -0.1311e+06;
    ionoParams.beta3 = 0.1049e+07;

    std::vector<GPSOpenCl::GpsEphemeris> ephemerides(4);
    std::vector<double> measuredRanges(4);

    for (int i = 0; i < 4; i++)
    {
        ephemerides[i].svId = i + 1;
        ephemerides[i].toe = 0.0;
        ephemerides[i].toc = 0.0;
        ephemerides[i].sqrtA = 5153.6;
        ephemerides[i].e = 0.001;
        ephemerides[i].M0 = i * 1.5;
        ephemerides[i].deltaN = 0.0;
        ephemerides[i].i0 = 0.95;
        ephemerides[i].idot = 0.0;
        ephemerides[i].omega0 = i * 1.57;
        ephemerides[i].omegaDot = 0.0;
        ephemerides[i].omega = 0.0;
        ephemerides[i].Cuc = ephemerides[i].Cus = ephemerides[i].Crc = ephemerides[i].Crs = ephemerides[i].Cic =
            ephemerides[i].Cis = 0.0;
        ephemerides[i].af0 = ephemerides[i].af1 = ephemerides[i].af2 = 0.0;

        GPSOpenCl::SatelliteOrbit orbit = GPSOpenCl::PVTSolver::computeSatelliteOrbit(ephemerides[i], 45000.0);
        double dx = orbit.position.x - rxEcef.x;
        double dy = orbit.position.y - rxEcef.y;
        double dz = orbit.position.z - rxEcef.z;
        double range = std::sqrt(dx * dx + dy * dy + dz * dz);

        double travelTime = range / c;
        double sagnacX = orbit.position.x + omegaE * travelTime * orbit.position.y;
        double sagnacY = orbit.position.y - omegaE * travelTime * orbit.position.x;

        dx = sagnacX - rxEcef.x;
        dy = sagnacY - rxEcef.y;
        dz = orbit.position.z - rxEcef.z;
        measuredRanges[i] = std::sqrt(dx * dx + dy * dy + dz * dz);

        GPSOpenCl::GeodeticPosition rxGeo = GPSOpenCl::PVTSolver::ecefToWgs84(rxEcef);
        GPSOpenCl::AtmosphericOutput atmo = GPSOpenCl::AtmosphericCorrections::computeCorrections(
            ephemerides[i].svId, rxGeo, rxEcef, orbit.position, 45000.0, ionoParams);
        measuredRanges[i] += atmo.ionoDelayMeters + atmo.tropoDelayMeters;
    }

    std::vector<double> transmitTimes(4, 45000.0);

    GPSOpenCl::PVTSolver correctedSolver;
    correctedSolver.setIonosphericParams(ionoParams);
    GPSOpenCl::ReceiverPvtSolution correctedSolution;
    EXPECT_TRUE(correctedSolver.solvePosition(ephemerides, measuredRanges, transmitTimes, correctedSolution));
    EXPECT_NEAR(correctedSolution.ecefPosition.x, rxEcef.x, 1.0);
    EXPECT_NEAR(correctedSolution.ecefPosition.y, rxEcef.y, 1.0);
    EXPECT_NEAR(correctedSolution.ecefPosition.z, rxEcef.z, 1.0);

    GPSOpenCl::PVTSolver uncorrectedSolver;
    GPSOpenCl::ReceiverPvtSolution uncorrectedSolution;
    uncorrectedSolver.solvePosition(ephemerides, measuredRanges, transmitTimes, uncorrectedSolution);
    double uncorrectedError = std::sqrt(std::pow(uncorrectedSolution.ecefPosition.x - rxEcef.x, 2) +
                                        std::pow(uncorrectedSolution.ecefPosition.y - rxEcef.y, 2) +
                                        std::pow(uncorrectedSolution.ecefPosition.z - rxEcef.z, 2));
    EXPECT_GT(uncorrectedError, 1.0);
}

TEST(PVTSolverTest, ElevationWeightingTrustsHighElevationSatelliteMoreThanLowElevation)
{
    GPSOpenCl::EcefPosition rxEcef;
    rxEcef.x = 4239828.0;
    rxEcef.y = 2447866.0;
    rxEcef.z = 4078438.0;

    const double c = 299792458.0;
    const double omegaE = 7.2921151467e-5;
    const int numSats = 6;

    std::vector<GPSOpenCl::GpsEphemeris> ephemerides(numSats);
    std::vector<double> measuredRanges(numSats);
    std::vector<double> elevations(numSats);

    for (int i = 0; i < numSats; i++)
    {
        ephemerides[i].svId = i + 1;
        ephemerides[i].toe = 0.0;
        ephemerides[i].toc = 0.0;
        ephemerides[i].sqrtA = 5153.6;
        ephemerides[i].e = 0.001;
        ephemerides[i].M0 = i * 1.1;
        ephemerides[i].deltaN = 0.0;
        ephemerides[i].i0 = 0.3 + i * 0.15;
        ephemerides[i].idot = 0.0;
        ephemerides[i].omega0 = i * 1.05;
        ephemerides[i].omegaDot = 0.0;
        ephemerides[i].omega = 0.0;
        ephemerides[i].Cuc = ephemerides[i].Cus = ephemerides[i].Crc = ephemerides[i].Crs = ephemerides[i].Cic =
            ephemerides[i].Cis = 0.0;
        ephemerides[i].af0 = ephemerides[i].af1 = ephemerides[i].af2 = 0.0;

        GPSOpenCl::SatelliteOrbit orbit = GPSOpenCl::PVTSolver::computeSatelliteOrbit(ephemerides[i], 0.0);
        double dx = orbit.position.x - rxEcef.x;
        double dy = orbit.position.y - rxEcef.y;
        double dz = orbit.position.z - rxEcef.z;
        double range = std::sqrt(dx * dx + dy * dy + dz * dz);

        double travelTime = range / c;
        double sagnacX = orbit.position.x + omegaE * travelTime * orbit.position.y;
        double sagnacY = orbit.position.y - omegaE * travelTime * orbit.position.x;

        dx = sagnacX - rxEcef.x;
        dy = sagnacY - rxEcef.y;
        dz = orbit.position.z - rxEcef.z;
        measuredRanges[i] = std::sqrt(dx * dx + dy * dy + dz * dz);

        double az = 0.0, el = 0.0;
        GPSOpenCl::AtmosphericCorrections::computeAzimuthElevation(rxEcef, orbit.position, az, el);
        elevations[i] = el;
    }

    int highIdx = 0, lowIdx = 0;
    for (int i = 1; i < numSats; i++)
    {
        if (elevations[i] > elevations[highIdx]) highIdx = i;
        if (elevations[i] < elevations[lowIdx]) lowIdx = i;
    }
    ASSERT_NE(highIdx, lowIdx) << "Need distinct high/low elevation satellites for this test to be meaningful";
    ASSERT_GT(elevations[highIdx] - elevations[lowIdx], 20.0)
        << "Test satellite geometry does not produce enough elevation spread to be meaningful. "
        << "high=" << elevations[highIdx] << " low=" << elevations[lowIdx];

    const double injectedBiasMeters = 80.0;
    std::vector<double> transmitTimes(numSats, 0.0);

    std::vector<double> rangesHighCorrupted = measuredRanges;
    rangesHighCorrupted[static_cast<size_t>(highIdx)] += injectedBiasMeters;
    GPSOpenCl::PVTSolver solverHigh;
    GPSOpenCl::ReceiverPvtSolution solutionHigh;
    ASSERT_TRUE(solverHigh.solvePosition(ephemerides, rangesHighCorrupted, transmitTimes, solutionHigh));
    double errorHigh = std::sqrt(std::pow(solutionHigh.ecefPosition.x - rxEcef.x, 2) +
                                 std::pow(solutionHigh.ecefPosition.y - rxEcef.y, 2) +
                                 std::pow(solutionHigh.ecefPosition.z - rxEcef.z, 2));

    std::vector<double> rangesLowCorrupted = measuredRanges;
    rangesLowCorrupted[static_cast<size_t>(lowIdx)] += injectedBiasMeters;
    GPSOpenCl::PVTSolver solverLow;
    GPSOpenCl::ReceiverPvtSolution solutionLow;
    ASSERT_TRUE(solverLow.solvePosition(ephemerides, rangesLowCorrupted, transmitTimes, solutionLow));
    double errorLow = std::sqrt(std::pow(solutionLow.ecefPosition.x - rxEcef.x, 2) +
                                std::pow(solutionLow.ecefPosition.y - rxEcef.y, 2) +
                                std::pow(solutionLow.ecefPosition.z - rxEcef.z, 2));

    EXPECT_GT(errorHigh - errorLow, 30.0)
        << "Expected an equal-size range error on the high-elevation (more trusted) satellite to pull "
        << "the weighted solution noticeably further off than the same-size error on the low-elevation "
        << "satellite -- errorHigh=" << errorHigh << "m errorLow=" << errorLow << "m";
}

TEST(PVTSolverTest, ComputeReceiverTimeIsConsistentAcrossSatellites)
{
    GPSOpenCl::EcefPosition rxEcef;
    rxEcef.x = 4239828.0;
    rxEcef.y = 2447866.0;
    rxEcef.z = 4078438.0;

    const double c = 299792458.0;
    const int numSats = 6;
    const double trueNowTow = 100.0;

    std::vector<GPSOpenCl::GpsEphemeris> ephemerides(numSats);
    std::vector<double> transmitTimes(numSats);

    for (int i = 0; i < numSats; i++)
    {
        ephemerides[i].svId = i + 1;
        ephemerides[i].toe = 0.0;
        ephemerides[i].toc = 0.0;
        ephemerides[i].sqrtA = 5153.6;
        ephemerides[i].e = 0.001;
        ephemerides[i].M0 = i * 1.1;
        ephemerides[i].deltaN = 0.0;
        ephemerides[i].i0 = 0.85 + (i * 0.05);
        ephemerides[i].idot = 0.0;
        ephemerides[i].omega0 = i * 1.05;
        ephemerides[i].omegaDot = 0.0;
        ephemerides[i].omega = 0.0;
        ephemerides[i].Cuc = ephemerides[i].Cus = ephemerides[i].Crc = ephemerides[i].Crs = ephemerides[i].Cic =
            ephemerides[i].Cis = 0.0;
        ephemerides[i].af0 = ephemerides[i].af1 = ephemerides[i].af2 = 0.0;

        double transitTime = 0.075;
        for (int iter = 0; iter < 6; iter++)
        {
            double candidateTransmitTime = trueNowTow - transitTime;
            GPSOpenCl::SatelliteOrbit orbit =
                GPSOpenCl::PVTSolver::computeSatelliteOrbit(ephemerides[i], candidateTransmitTime);
            double dx = orbit.position.x - rxEcef.x;
            double dy = orbit.position.y - rxEcef.y;
            double dz = orbit.position.z - rxEcef.z;
            transitTime = std::sqrt((dx * dx) + (dy * dy) + (dz * dz)) / c;
        }
        transmitTimes[static_cast<size_t>(i)] = trueNowTow - transitTime;
    }

    double receiverTime = GPSOpenCl::PVTSolver::computeReceiverTime(ephemerides, transmitTimes, rxEcef);

    EXPECT_NEAR(receiverTime, trueNowTow, 1e-6)
        << "computeReceiverTime should reconstruct the true 'now' instant (to numerical precision) "
        << "when every transmitTimeSeconds[i] is the true transmit time of the currently-arriving "
        << "signal and referenceEcef is the true receiver position -- got " << receiverTime << " expected "
        << trueNowTow;
}

TEST(PVTSolverTest, SvClockBiasDoesNotDegradePositionSolution)
{
    GPSOpenCl::EcefPosition rxEcef;
    rxEcef.x = 4239828.0;
    rxEcef.y = 2447866.0;
    rxEcef.z = 4078438.0;

    const double c = 299792458.0;
    const double omegaE = 7.2921151467e-5;
    const int numSats = 6;

    const double af0Values[numSats] = {-0.0005, -0.0002, 0.0001, 0.0003, 0.0006, 0.0009};

    std::vector<GPSOpenCl::GpsEphemeris> ephemerides(numSats);
    std::vector<double> measuredRanges(numSats);
    std::vector<double> transmitTimes(static_cast<size_t>(numSats), 45000.0);

    for (int i = 0; i < numSats; i++)
    {
        ephemerides[i].svId = i + 1;
        ephemerides[i].toe = 45000.0;
        ephemerides[i].toc = 45000.0;
        ephemerides[i].sqrtA = 5153.6;
        ephemerides[i].e = 0.001;
        ephemerides[i].M0 = i * 1.1;
        ephemerides[i].deltaN = 0.0;
        ephemerides[i].i0 = 0.85 + (i * 0.05);
        ephemerides[i].idot = 0.0;
        ephemerides[i].omega0 = i * 1.05;
        ephemerides[i].omegaDot = 0.0;
        ephemerides[i].omega = 0.0;
        ephemerides[i].Cuc = ephemerides[i].Cus = ephemerides[i].Crc = ephemerides[i].Crs = ephemerides[i].Cic =
            ephemerides[i].Cis = 0.0;
        ephemerides[i].af0 = af0Values[i];
        ephemerides[i].af1 = 0.0;
        ephemerides[i].af2 = 0.0;
        ephemerides[i].tgd = 0.0;

        GPSOpenCl::SatelliteOrbit orbit =
            GPSOpenCl::PVTSolver::computeSatelliteOrbit(ephemerides[i], transmitTimes[static_cast<size_t>(i)]);
        double dx = orbit.position.x - rxEcef.x;
        double dy = orbit.position.y - rxEcef.y;
        double dz = orbit.position.z - rxEcef.z;
        double range = std::sqrt(dx * dx + dy * dy + dz * dz);

        double travelTime = range / c;
        double sagnacX = orbit.position.x + omegaE * travelTime * orbit.position.y;
        double sagnacY = orbit.position.y - omegaE * travelTime * orbit.position.x;

        dx = sagnacX - rxEcef.x;
        dy = sagnacY - rxEcef.y;
        dz = orbit.position.z - rxEcef.z;
        range = std::sqrt(dx * dx + dy * dy + dz * dz);

        measuredRanges[static_cast<size_t>(i)] = range - c * orbit.clockBias;

        GPSOpenCl::GeodeticPosition rxGeo = GPSOpenCl::PVTSolver::ecefToWgs84(rxEcef);
        GPSOpenCl::AtmosphericInput noIonoParams{};
        GPSOpenCl::AtmosphericOutput atmo = GPSOpenCl::AtmosphericCorrections::computeCorrections(
            ephemerides[i].svId, rxGeo, rxEcef, orbit.position, transmitTimes[static_cast<size_t>(i)], noIonoParams);
        measuredRanges[static_cast<size_t>(i)] += atmo.ionoDelayMeters + atmo.tropoDelayMeters;
    }

    GPSOpenCl::PVTSolver solver;
    GPSOpenCl::ReceiverPvtSolution solution;
    ASSERT_TRUE(solver.solvePosition(ephemerides, measuredRanges, transmitTimes, solution));

    EXPECT_NEAR(solution.ecefPosition.x, rxEcef.x, 1.0)
        << "Per-satellite SV clock bias should cancel exactly against solvePosition's own "
        << "c*orbit.clockBias correction and not leak into the position estimate";
    EXPECT_NEAR(solution.ecefPosition.y, rxEcef.y, 1.0);
    EXPECT_NEAR(solution.ecefPosition.z, rxEcef.z, 1.0);
}
}
