#include "GPSOpenClPVTSolver.h"
#include "GPSOpenClAtmosphericCorrections.h"

#include "gtest/gtest.h"
#include <cmath>

namespace GPSOpenClTest
{
TEST(PVTSolverTest, EcefToWgs84Equator)
{
    GPSOpenCl::EcefPosition ecef;
    ecef.x = 6378137.0; // Point on equator on prime meridian
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
    ephem.sqrtA = 5153.6; // ~26560 km orbital radius
    ephem.e = 0.01;
    ephem.M0 = 0.0;
    ephem.deltaN = 0.0;
    ephem.i0 = 0.96; // ~55 degrees
    ephem.idot = 0.0;
    ephem.omega0 = 0.0;
    ephem.omegaDot = 0.0;
    ephem.omega = 0.0;
    ephem.Cuc = ephem.Cus = ephem.Crc = ephem.Crs = ephem.Cic = ephem.Cis = 0.0;
    ephem.af0 = ephem.af1 = ephem.af2 = 0.0;

    GPSOpenCl::SatelliteOrbit orbit = GPSOpenCl::PVTSolver::computeSatelliteOrbit(ephem, 1000.0);

    double radius = std::sqrt(orbit.position.x * orbit.position.x +
                              orbit.position.y * orbit.position.y +
                              orbit.position.z * orbit.position.z);

    double expectedRadius = ephem.sqrtA * ephem.sqrtA * (1.0 - ephem.e);
    EXPECT_NEAR(radius, expectedRadius, 10.0);
}

TEST(PVTSolverTest, SolvePositionSynthesized)
{
    // Receiver position at latitude 40 N, longitude 30 E, altitude 100 m
    GPSOpenCl::EcefPosition rxEcef;
    rxEcef.x = 4239828.0;
    rxEcef.y = 2447866.0;
    rxEcef.z = 4078438.0;

    const double c = 299792458.0;
    const double omegaE = 7.2921151467e-5;

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
        ephemerides[i].Cuc = ephemerides[i].Cus = ephemerides[i].Crc = ephemerides[i].Crs = ephemerides[i].Cic = ephemerides[i].Cis = 0.0;
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

        // Real receivers measure geometric range plus tropospheric delay; inject the same delay
        // the solver now corrects for, so this test validates that correction recovers the true
        // position instead of just checking pure, atmosphere-free geometry.
        double azDeg = 0.0, elDeg = 0.0;
        GPSOpenCl::AtmosphericCorrections::computeAzimuthElevation(rxEcef, orbit.position, azDeg, elDeg);
        double rxAltitude = GPSOpenCl::PVTSolver::ecefToWgs84(rxEcef).altitude;
        measuredRanges[i] += GPSOpenCl::AtmosphericCorrections::saastamoinenTroposphericDelay(rxAltitude, elDeg);
    }

    GPSOpenCl::PVTSolver solver;
    GPSOpenCl::ReceiverPvtSolution solution;
    std::vector<double> transmitTimes(4, 0.0);
    bool success = solver.solvePosition(ephemerides, measuredRanges, transmitTimes, solution);

    EXPECT_TRUE(success);
    EXPECT_TRUE(solution.isValid);
    EXPECT_NEAR(solution.ecefPosition.x, rxEcef.x, 1.0);
    EXPECT_NEAR(solution.ecefPosition.y, rxEcef.y, 1.0);
    EXPECT_NEAR(solution.ecefPosition.z, rxEcef.z, 1.0);
}

TEST(PVTSolverTest, StructOverloadResetsOutputOnFailure)
{
    std::vector<GPSOpenCl::NavDecoderOutput> outputs(2); // fewer than the default minSatellites (4)
    std::vector<double> measuredRanges(2, 20000000.0);
    std::vector<double> transmitTimes(2, 0.0);

    GPSOpenCl::PVTSolver solver;
    GPSOpenCl::PvtSolverOutput outputSolution{};
    outputSolution.isValid = 1; // simulate a stale fix from a previous, successful call
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
    std::vector<GPSOpenCl::GpsEphemeris> ephemerides(4); // fewer than the configured minSatellites
    std::vector<double> measuredRanges(4, 20000000.0);
    std::vector<double> transmitTimes(4, 0.0);
    GPSOpenCl::ReceiverPvtSolution solution;

    bool success = solver.solvePosition(ephemerides, measuredRanges, transmitTimes, solution);

    EXPECT_FALSE(success);
    EXPECT_FALSE(solution.isValid);
}

TEST(PVTSolverTest, MaxPseudorangeErrGateRejectsOutlierWithRedundantSatellites)
{
    // RAIM-style residual checks need redundancy: with exactly 4 satellites the system is exactly
    // determined and a bad measurement is absorbed with ~zero residual, so this needs a 5th satellite.
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
        ephemerides[i].Cuc = ephemerides[i].Cus = ephemerides[i].Crc = ephemerides[i].Crs = ephemerides[i].Cic = ephemerides[i].Cis = 0.0;
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

        // Inject realistic tropospheric delay (no corruption) so the "clean" baseline below also
        // proves normal, uncorrected-by-the-caller atmospheric delay does not by itself trip the
        // outlier gate now that solvePosition corrects for it internally.
        double azDeg = 0.0, elDeg = 0.0;
        GPSOpenCl::AtmosphericCorrections::computeAzimuthElevation(rxEcef, orbit.position, azDeg, elDeg);
        double rxAltitude = GPSOpenCl::PVTSolver::ecefToWgs84(rxEcef).altitude;
        measuredRanges[i] += GPSOpenCl::AtmosphericCorrections::saastamoinenTroposphericDelay(rxAltitude, elDeg);
    }

    GPSOpenCl::PVTSolver solver;
    std::vector<double> transmitTimes(numSats, 0.0);

    GPSOpenCl::ReceiverPvtSolution cleanSolution;
    EXPECT_TRUE(solver.solvePosition(ephemerides, measuredRanges, transmitTimes, cleanSolution));

    // Corrupt one measurement far beyond the default 100 m outlier gate.
    measuredRanges[2] += 10000.0;
    GPSOpenCl::ReceiverPvtSolution corruptedSolution;
    bool success = solver.solvePosition(ephemerides, measuredRanges, transmitTimes, corruptedSolution);

    EXPECT_FALSE(success);
}
} // namespace GPSOpenClTest
