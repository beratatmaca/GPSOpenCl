#include "GPSOpenClPVTSolver.h"

#include "GPSOpenClAtmosphericCorrections.h"

#include <cmath>
#include <iostream>

using namespace GPSOpenCl;

PVTSolver::PVTSolver()
    : m_inputConfig{STRUCT_VERSION_1, 4, 100.0}
{
}

PVTSolver::PVTSolver(const PvtSolverInput &input)
    : m_inputConfig(input)
{
}

PVTSolver::~PVTSolver()
{
}

PvtSolverOutput PVTSolver::solutionToOutput(const ReceiverPvtSolution &sol)
{
    PvtSolverOutput out{};
    out.structVersion = STRUCT_VERSION_1;
    out.ecefX = sol.ecefPosition.x;
    out.ecefY = sol.ecefPosition.y;
    out.ecefZ = sol.ecefPosition.z;
    out.latitude = sol.geodeticPosition.latitude;
    out.longitude = sol.geodeticPosition.longitude;
    out.altitude = sol.geodeticPosition.altitude;
    out.clockBiasMeters = sol.clockBiasMeters;
    out.clockBiasSeconds = sol.clockBiasSeconds;
    out.dopGDOP = sol.dopGDOP;
    out.dopPDOP = sol.dopPDOP;
    out.dopHDOP = sol.dopHDOP;
    out.dopVDOP = sol.dopVDOP;
    out.isValid = sol.isValid ? 1 : 0;
    return out;
}

ReceiverPvtSolution PVTSolver::outputToSolution(const PvtSolverOutput &out)
{
    ReceiverPvtSolution sol{};
    sol.ecefPosition.x = out.ecefX;
    sol.ecefPosition.y = out.ecefY;
    sol.ecefPosition.z = out.ecefZ;
    sol.geodeticPosition.latitude = out.latitude;
    sol.geodeticPosition.longitude = out.longitude;
    sol.geodeticPosition.altitude = out.altitude;
    sol.clockBiasMeters = out.clockBiasMeters;
    sol.clockBiasSeconds = out.clockBiasSeconds;
    sol.dopGDOP = out.dopGDOP;
    sol.dopPDOP = out.dopPDOP;
    sol.dopHDOP = out.dopHDOP;
    sol.dopVDOP = out.dopVDOP;
    sol.isValid = (out.isValid != 0);
    return sol;
}

SatelliteOrbit PVTSolver::computeSatelliteOrbit(const NavDecoderOutput &navOut, double t)
{
    GpsEphemeris ephem = NavigationDecoder::outputToEphemeris(navOut);
    return computeSatelliteOrbit(ephem, t);
}

SatelliteOrbit PVTSolver::computeSatelliteOrbit(const GpsEphemeris &ephem, double t)
{
    SatelliteOrbit orbit;
    orbit.svId = ephem.svId;

    const double mu = 3.986005e14;
    const double omegaE = 7.2921151467e-5;
    const double F = -4.442807633e-10;

    double A = ephem.sqrtA * ephem.sqrtA;
    double n0 = std::sqrt(mu / (A * A * A));

    double tk = t - ephem.toe;
    if (tk > 302400.0) tk -= 604800.0;
    if (tk < -302400.0) tk += 604800.0;

    double n = n0 + ephem.deltaN;
    double Mk = ephem.M0 + n * tk;


    double Ek = Mk;
    for (int iter = 0; iter < 10; iter++)
    {
        double diff = Ek - ephem.e * std::sin(Ek) - Mk;
        double derivative = 1.0 - ephem.e * std::cos(Ek);
        Ek -= diff / derivative;
    }


    orbit.relCorr = F * ephem.e * ephem.sqrtA * std::sin(Ek);


    double dtT = t - ephem.toc;
    if (dtT > 302400.0) dtT -= 604800.0;
    if (dtT < -302400.0) dtT += 604800.0;

    orbit.clockBias = ephem.af0 + ephem.af1 * dtT + ephem.af2 * dtT * dtT + orbit.relCorr;


    double sinEk = std::sin(Ek);
    double cosEk = std::cos(Ek);
    double sinVk = (std::sqrt(1.0 - ephem.e * ephem.e) * sinEk) / (1.0 - ephem.e * cosEk);
    double cosVk = (cosEk - ephem.e) / (1.0 - ephem.e * cosEk);
    double vk = std::atan2(sinVk, cosVk);


    double phiK = vk + ephem.omega;


    double sin2Phi = std::sin(2.0 * phiK);
    double cos2Phi = std::cos(2.0 * phiK);

    double deltaU = ephem.Cus * sin2Phi + ephem.Cuc * cos2Phi;
    double deltaR = ephem.Crs * sin2Phi + ephem.Crc * cos2Phi;
    double deltaI = ephem.Cis * sin2Phi + ephem.Cic * cos2Phi;

    double uk = phiK + deltaU;
    double rk = A * (1.0 - ephem.e * cosEk) + deltaR;
    double ik = ephem.i0 + deltaI + ephem.idot * tk;


    double xkPrime = rk * std::cos(uk);
    double ykPrime = rk * std::sin(uk);


    double omegaK = ephem.omega0 + (ephem.omegaDot - omegaE) * tk - omegaE * ephem.toe;


    orbit.position.x = xkPrime * std::cos(omegaK) - ykPrime * std::cos(ik) * std::sin(omegaK);
    orbit.position.y = xkPrime * std::sin(omegaK) + ykPrime * std::cos(ik) * std::cos(omegaK);
    orbit.position.z = ykPrime * std::sin(ik);

    return orbit;
}

GeodeticPosition PVTSolver::ecefToWgs84(const EcefPosition &ecef)
{
    GeodeticPosition geo;
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double b = a * (1.0 - f);
    const double e2 = (a * a - b * b) / (a * a);
    const double ep2 = (a * a - b * b) / (b * b);

    double p = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
    double theta = std::atan2(ecef.z * a, p * b);

    double sinTheta = std::sin(theta);
    double cosTheta = std::cos(theta);

    double latRad = std::atan2(ecef.z + ep2 * b * sinTheta * sinTheta * sinTheta,
                               p - e2 * a * cosTheta * cosTheta * cosTheta);
    double lonRad = std::atan2(ecef.y, ecef.x);

    double sinLat = std::sin(latRad);
    double N = a / std::sqrt(1.0 - e2 * sinLat * sinLat);

    geo.latitude = latRad * 180.0 / M_PI;
    geo.longitude = lonRad * 180.0 / M_PI;
    geo.altitude = (p / std::cos(latRad)) - N;

    return geo;
}

bool PVTSolver::solvePosition(const std::vector<GpsEphemeris> &ephemerides,
                              const std::vector<double> &measuredPseudoranges,
                              const std::vector<double> &transmitTimesSeconds,
                              ReceiverPvtSolution &solution)
{
    solution.isValid = false;
    size_t numSats = ephemerides.size();
    if (numSats < static_cast<size_t>(m_inputConfig.minSatellites) || measuredPseudoranges.size() < numSats ||
        transmitTimesSeconds.size() < numSats)
    {
        return false;
    }

    const double c = 299792458.0;
    const double omegaE = 7.2921151467e-5;

    std::vector<SatelliteOrbit> orbits(numSats);
    std::vector<double> correctedRanges(numSats);

    for (size_t i = 0; i < numSats; i++)
    {
        orbits[i] = computeSatelliteOrbit(ephemerides[i], transmitTimesSeconds[i]);

        correctedRanges[i] = measuredPseudoranges[i] + c * orbits[i].clockBias;
    }


    double state[4] = {4180483.4, 851798.0, 4725999.8, 0.0};


    for (int iter = 0; iter < 15; iter++)
    {
        std::vector<std::vector<double>> H(numSats, std::vector<double>(4, 0.0));
        std::vector<double> deltaRho(numSats, 0.0);









        EcefPosition rxEcefEstimate{state[0], state[1], state[2]};
        double rxAltitudeEstimate = ecefToWgs84(rxEcefEstimate).altitude;

        for (size_t i = 0; i < numSats; i++)
        {
            double dx = orbits[i].position.x - state[0];
            double dy = orbits[i].position.y - state[1];
            double dz = orbits[i].position.z - state[2];
            double range = std::sqrt(dx * dx + dy * dy + dz * dz);


            double travelTime = range / c;
            double sagnacX = orbits[i].position.x + omegaE * travelTime * orbits[i].position.y;
            double sagnacY = orbits[i].position.y - omegaE * travelTime * orbits[i].position.x;

            dx = sagnacX - state[0];
            dy = sagnacY - state[1];
            dz = orbits[i].position.z - state[2];
            range = std::sqrt(dx * dx + dy * dy + dz * dz);

            double azDeg = 0.0, elDeg = 0.0;
            AtmosphericCorrections::computeAzimuthElevation(rxEcefEstimate, orbits[i].position, azDeg, elDeg);
            double tropoDelayMeters = AtmosphericCorrections::saastamoinenTroposphericDelay(rxAltitudeEstimate, elDeg);

            double predictedPseudorange = range + state[3];
            deltaRho[i] = (correctedRanges[i] - tropoDelayMeters) - predictedPseudorange;

            H[i][0] = -dx / range;
            H[i][1] = -dy / range;
            H[i][2] = -dz / range;
            H[i][3] = 1.0;
        }


        double HtH[4][4] = {{0}};
        double HtY[4] = {0};

        for (size_t i = 0; i < numSats; i++)
        {
            for (int r = 0; r < 4; r++)
            {
                HtY[r] += H[i][r] * deltaRho[i];
                for (int col = 0; col < 4; col++)
                {
                    HtH[r][col] += H[i][r] * H[i][col];
                }
            }
        }


        double A[4][8] = {{0}};
        for (int r = 0; r < 4; r++)
        {
            for (int col = 0; col < 4; col++)
            {
                A[r][col] = HtH[r][col];
            }
            A[r][r + 4] = 1.0;
        }

        for (int i = 0; i < 4; i++)
        {
            double pivot = A[i][i];
            if (std::fabs(pivot) < 1e-12) return false;
            for (int j = 0; j < 8; j++) A[i][j] /= pivot;
            for (int k = 0; k < 4; k++)
            {
                if (k != i)
                {
                    double factor = A[k][i];
                    for (int j = 0; j < 8; j++) A[k][j] -= factor * A[i][j];
                }
            }
        }

        double deltaX[4] = {0};
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                deltaX[i] += A[i][j + 4] * HtY[j];
            }
            state[i] += deltaX[i];
        }

        double stepNorm = std::sqrt(deltaX[0] * deltaX[0] + deltaX[1] * deltaX[1] + deltaX[2] * deltaX[2]);
        if (stepNorm < 1e-4)
        {
            double maxAbsResidual = 0.0;
            for (size_t i = 0; i < numSats; i++)
            {
                double absResidual = std::fabs(deltaRho[i]);
                if (absResidual > maxAbsResidual) maxAbsResidual = absResidual;
            }




            if (maxAbsResidual > m_inputConfig.maxPseudorangeErrMeters)
            {


                return false;
            }

            solution.ecefPosition.x = state[0];
            solution.ecefPosition.y = state[1];
            solution.ecefPosition.z = state[2];
            solution.clockBiasMeters = state[3];
            solution.clockBiasSeconds = state[3] / c;
            solution.geodeticPosition = ecefToWgs84(solution.ecefPosition);


            double invHtH[4][4];
            for (int r = 0; r < 4; r++)
                for (int col = 0; col < 4; col++)
                    invHtH[r][col] = A[r][col + 4];

            solution.dopGDOP = std::sqrt(invHtH[0][0] + invHtH[1][1] + invHtH[2][2] + invHtH[3][3]);
            solution.dopPDOP = std::sqrt(invHtH[0][0] + invHtH[1][1] + invHtH[2][2]);
            solution.dopHDOP = std::sqrt(invHtH[0][0] + invHtH[1][1]);
            solution.dopVDOP = std::sqrt(invHtH[2][2]);

            solution.isValid = true;
            return true;
        }
    }

    return false;
}

bool PVTSolver::solvePosition(const std::vector<NavDecoderOutput> &outputs,
                              const std::vector<double> &measuredPseudoranges,
                              const std::vector<double> &transmitTimesSeconds,
                              PvtSolverOutput &outputSolution)
{
    std::vector<GpsEphemeris> ephemerides;
    ephemerides.reserve(outputs.size());
    for (const auto &out : outputs)
    {
        ephemerides.push_back(NavigationDecoder::outputToEphemeris(out));
    }

    ReceiverPvtSolution sol{};
    bool res = solvePosition(ephemerides, measuredPseudoranges, transmitTimesSeconds, sol);


    outputSolution = solutionToOutput(sol);
    return res;
}
