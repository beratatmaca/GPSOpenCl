#include "GPSOpenClPVTSolver.h"

#include "GPSOpenClAtmosphericCorrections.h"

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace GPSOpenCl;

namespace
{
bool invert4x4(const double m[4][4], double inv[4][4])
{
    double a[4][8] = {{0}};
    for (int r = 0; r < 4; r++)
    {
        for (int c = 0; c < 4; c++)
        {
            a[r][c] = m[r][c];
        }
        a[r][r + 4] = 1.0;
    }

    for (int i = 0; i < 4; i++)
    {
        int pivotRow = i;
        double pivotAbs = std::fabs(a[i][i]);
        for (int k = i + 1; k < 4; k++)
        {
            if (std::fabs(a[k][i]) > pivotAbs)
            {
                pivotAbs = std::fabs(a[k][i]);
                pivotRow = k;
            }
        }
        if (pivotAbs < 1e-12)
        {
            return false;
        }
        if (pivotRow != i)
        {
            for (int j = 0; j < 8; j++)
            {
                std::swap(a[i][j], a[pivotRow][j]);
            }
        }
        const double pivot = a[i][i];
        for (int j = 0; j < 8; j++)
        {
            a[i][j] /= pivot;
        }
        for (int k = 0; k < 4; k++)
        {
            if (k != i)
            {
                const double factor = a[k][i];
                for (int j = 0; j < 8; j++)
                {
                    a[k][j] -= factor * a[i][j];
                }
            }
        }
    }

    for (int r = 0; r < 4; r++)
    {
        for (int c = 0; c < 4; c++)
        {
            inv[r][c] = a[r][c + 4];
        }
    }
    return true;
}
}

PVTSolver::PVTSolver() : m_inputConfig{STRUCT_VERSION_1, 4, 100.0}
{
}

PVTSolver::PVTSolver(const PvtSolverInput &input) : m_inputConfig(input)
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
    const GpsEphemeris ephem = NavigationDecoder::outputToEphemeris(navOut);
    return computeSatelliteOrbit(ephem, t);
}

SatelliteOrbit PVTSolver::computeSatelliteOrbit(const GpsEphemeris &ephem, double t)
{
    SatelliteOrbit orbit{};
    orbit.svId = ephem.svId;

    const double mu = 3.986005e14;
    const double omegaE = 7.2921151467e-5;
    const double F = -4.442807633e-10;

    const double A = ephem.sqrtA * ephem.sqrtA;
    const double n0 = std::sqrt(mu / (A * A * A));

    double dtT = t - ephem.toc;
    if (dtT > 302400.0)
    {
        dtT -= 604800.0;
    }
    if (dtT < -302400.0)
    {
        dtT += 604800.0;
    }
    const double polyClockBias = ephem.af0 + (ephem.af1 * dtT) + (ephem.af2 * dtT * dtT);

    double tk = t - polyClockBias - ephem.toe;
    if (tk > 302400.0)
    {
        tk -= 604800.0;
    }
    if (tk < -302400.0)
    {
        tk += 604800.0;
    }

    const double n = n0 + ephem.deltaN;
    const double Mk = ephem.M0 + (n * tk);

    double Ek = Mk;
    for (int iter = 0; iter < 10; iter++)
    {
        const double diff = Ek - (ephem.e * std::sin(Ek)) - Mk;
        const double derivative = 1.0 - (ephem.e * std::cos(Ek));
        Ek -= diff / derivative;
    }

    orbit.relCorr = F * ephem.e * ephem.sqrtA * std::sin(Ek);

    orbit.clockBias = polyClockBias + orbit.relCorr - ephem.tgd;

    const double sinEk = std::sin(Ek);
    const double cosEk = std::cos(Ek);
    const double sinVk = (std::sqrt(1.0 - (ephem.e * ephem.e)) * sinEk) / (1.0 - ephem.e * cosEk);
    const double cosVk = (cosEk - ephem.e) / (1.0 - ephem.e * cosEk);
    const double vk = std::atan2(sinVk, cosVk);

    const double phiK = vk + ephem.omega;

    const double sin2Phi = std::sin(2.0 * phiK);
    const double cos2Phi = std::cos(2.0 * phiK);

    const double deltaU = (ephem.Cus * sin2Phi) + (ephem.Cuc * cos2Phi);
    const double deltaR = (ephem.Crs * sin2Phi) + (ephem.Crc * cos2Phi);
    const double deltaI = (ephem.Cis * sin2Phi) + (ephem.Cic * cos2Phi);

    const double uk = phiK + deltaU;
    const double rk = (A * (1.0 - ephem.e * cosEk)) + deltaR;
    const double ik = ephem.i0 + deltaI + (ephem.idot * tk);

    const double xkPrime = rk * std::cos(uk);
    const double ykPrime = rk * std::sin(uk);

    const double omegaK = ephem.omega0 + ((ephem.omegaDot - omegaE) * tk) - (omegaE * ephem.toe);

    orbit.position.x = xkPrime * std::cos(omegaK) - ykPrime * std::cos(ik) * std::sin(omegaK);
    orbit.position.y = xkPrime * std::sin(omegaK) + ykPrime * std::cos(ik) * std::cos(omegaK);
    orbit.position.z = ykPrime * std::sin(ik);

    return orbit;
}

GeodeticPosition PVTSolver::ecefToWgs84(const EcefPosition &ecef)
{
    GeodeticPosition geo{};
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double b = a * (1.0 - f);
    const double e2 = (a * a - b * b) / (a * a);
    const double ep2 = (a * a - b * b) / (b * b);

    const double p = std::sqrt((ecef.x * ecef.x) + (ecef.y * ecef.y));
    const double theta = std::atan2(ecef.z * a, p * b);

    const double sinTheta = std::sin(theta);
    const double cosTheta = std::cos(theta);

    const double latRad =
        std::atan2(ecef.z + (ep2 * b * sinTheta * sinTheta * sinTheta), p - (e2 * a * cosTheta * cosTheta * cosTheta));
    const double lonRad = std::atan2(ecef.y, ecef.x);

    const double sinLat = std::sin(latRad);
    const double N = a / std::sqrt(1.0 - (e2 * sinLat * sinLat));

    geo.latitude = latRad * 180.0 / M_PI;
    geo.longitude = lonRad * 180.0 / M_PI;
    geo.altitude = (p / std::cos(latRad)) - N;

    return geo;
}

double PVTSolver::computeReceiverTime(const std::vector<GpsEphemeris> &ephemerides,
                                      const std::vector<double> &transmitTimesSeconds,
                                      const EcefPosition &referenceEcef)
{
    if (transmitTimesSeconds.empty() || ephemerides.size() < transmitTimesSeconds.size())
    {
        return 0.0;
    }

    const double c = 299792458.0;
    double averageTransmitTime = 0.0;
    double averageTransitTimeSec = 0.0;
    for (size_t i = 0; i < transmitTimesSeconds.size(); i++)
    {
        const SatelliteOrbit orbit = computeSatelliteOrbit(ephemerides[i], transmitTimesSeconds[i]);
        const double dx = orbit.position.x - referenceEcef.x;
        const double dy = orbit.position.y - referenceEcef.y;
        const double dz = orbit.position.z - referenceEcef.z;

        averageTransmitTime += transmitTimesSeconds[i];
        averageTransitTimeSec += std::sqrt(dx * dx + dy * dy + dz * dz) / c;
    }
    averageTransmitTime /= static_cast<double>(transmitTimesSeconds.size());
    averageTransitTimeSec /= static_cast<double>(transmitTimesSeconds.size());

    return averageTransmitTime + averageTransitTimeSec;
}

bool PVTSolver::solvePosition(const std::vector<GpsEphemeris> &ephemerides,
                              const std::vector<double> &measuredPseudoranges,
                              const std::vector<double> &transmitTimesSeconds,
                              ReceiverPvtSolution &solution)
{
    solution.isValid = false;
    const size_t numSats = ephemerides.size();
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

    double state[4] = {m_referenceEcef.x, m_referenceEcef.y, m_referenceEcef.z, 0.0};

    std::vector<std::vector<double>> H(numSats, std::vector<double>(4, 0.0));
    std::vector<double> deltaRho(numSats, 0.0);
    std::vector<double> weight(numSats, 1.0);

    for (int iter = 0; iter < 15; iter++)
    {

        const EcefPosition rxEcefEstimate{state[0], state[1], state[2]};
        const GeodeticPosition rxGeodeticEstimate = ecefToWgs84(rxEcefEstimate);

        for (size_t i = 0; i < numSats; i++)
        {
            double dx = orbits[i].position.x - state[0];
            double dy = orbits[i].position.y - state[1];
            double dz = orbits[i].position.z - state[2];
            double range = std::sqrt((dx * dx) + (dy * dy) + (dz * dz));

            const double travelTime = range / c;
            const double sagnacX = orbits[i].position.x + (omegaE * travelTime * orbits[i].position.y);
            const double sagnacY = orbits[i].position.y - (omegaE * travelTime * orbits[i].position.x);

            dx = sagnacX - state[0];
            dy = sagnacY - state[1];
            dz = orbits[i].position.z - state[2];
            range = std::sqrt((dx * dx) + (dy * dy) + (dz * dz));

            const AtmosphericOutput atmo = AtmosphericCorrections::computeCorrections(orbits[i].svId,
                                                                                      rxGeodeticEstimate,
                                                                                      rxEcefEstimate,
                                                                                      orbits[i].position,
                                                                                      transmitTimesSeconds[i],
                                                                                      m_ionoParams);

            const double predictedPseudorange = range + state[3];
            deltaRho[i] = (correctedRanges[i] - atmo.ionoDelayMeters - atmo.tropoDelayMeters) - predictedPseudorange;

            H[i][0] = -dx / range;
            H[i][1] = -dy / range;
            H[i][2] = -dz / range;
            H[i][3] = 1.0;

            const double elevRad = std::clamp(atmo.elevationDeg, 5.0, 90.0) * M_PI / 180.0;
            const double sinElev = std::sin(elevRad);
            weight[i] = sinElev * sinElev;
        }

        double HtH[4][4] = {{0}};
        double HtHUnweighted[4][4] = {{0}};
        double HtY[4] = {0};

        for (size_t i = 0; i < numSats; i++)
        {
            for (int r = 0; r < 4; r++)
            {
                HtY[r] += weight[i] * H[i][r] * deltaRho[i];
                for (int col = 0; col < 4; col++)
                {
                    HtH[r][col] += weight[i] * H[i][r] * H[i][col];
                    HtHUnweighted[r][col] += H[i][r] * H[i][col];
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
            int pivotRow = i;
            double pivotAbs = std::fabs(A[i][i]);
            for (int k = i + 1; k < 4; k++)
            {
                if (std::fabs(A[k][i]) > pivotAbs)
                {
                    pivotAbs = std::fabs(A[k][i]);
                    pivotRow = k;
                }
            }
            if (pivotAbs < 1e-12)
            {
                return false;
            }
            if (pivotRow != i)
            {
                for (int j = 0; j < 8; j++)
                {
                    const double tmp = A[i][j];
                    A[i][j] = A[pivotRow][j];
                    A[pivotRow][j] = tmp;
                }
            }
            const double pivot = A[i][i];
            for (int j = 0; j < 8; j++)
            {
                A[i][j] /= pivot;
            }
            for (int k = 0; k < 4; k++)
            {
                if (k != i)
                {
                    const double factor = A[k][i];
                    for (int j = 0; j < 8; j++)
                    {
                        A[k][j] -= factor * A[i][j];
                    }
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

        const double stepNorm = std::sqrt((deltaX[0] * deltaX[0]) + (deltaX[1] * deltaX[1]) + (deltaX[2] * deltaX[2]));
        if (stepNorm < 1e-4)
        {
            double maxAbsResidual = 0.0;
            for (size_t i = 0; i < numSats; i++)
            {
                const double absResidual = std::fabs(deltaRho[i]);
                maxAbsResidual = std::max(absResidual, maxAbsResidual);
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

            double invHtH[4][4] = {{0}};
            if (!invert4x4(HtHUnweighted, invHtH))
            {
                solution.dopGDOP = 0.0;
                solution.dopPDOP = 0.0;
                solution.dopHDOP = 0.0;
                solution.dopVDOP = 0.0;
                solution.isValid = true;
                m_referenceEcef = solution.ecefPosition;
                return true;
            }

            solution.dopGDOP = std::sqrt(invHtH[0][0] + invHtH[1][1] + invHtH[2][2] + invHtH[3][3]);
            solution.dopPDOP = std::sqrt(invHtH[0][0] + invHtH[1][1] + invHtH[2][2]);

            const double latRad = solution.geodeticPosition.latitude * M_PI / 180.0;
            const double lonRad = solution.geodeticPosition.longitude * M_PI / 180.0;
            double sinLat = std::sin(latRad);
            double cosLat = std::cos(latRad);
            const double sinLon = std::sin(lonRad);
            double cosLon = std::cos(lonRad);

            const double enuRotation[3][3] = {
                {-sinLon,          cosLon,           0.0   },
                {-sinLat * cosLon, -sinLat * sinLon, cosLat},
                {cosLat * cosLon,  cosLat * sinLon,  sinLat}
            };

            double enuCovariance[3][3] = {{0.0}};
            for (int r = 0; r < 3; r++)
            {
                for (int col = 0; col < 3; col++)
                {
                    double sum = 0.0;
                    for (int a = 0; a < 3; a++)
                    {
                        for (int b = 0; b < 3; b++)
                        {
                            sum += enuRotation[r][a] * invHtH[a][b] * enuRotation[col][b];
                        }
                    }
                    enuCovariance[r][col] = sum;
                }
            }

            solution.dopHDOP = std::sqrt(enuCovariance[0][0] + enuCovariance[1][1]);
            solution.dopVDOP = std::sqrt(enuCovariance[2][2]);

            solution.isValid = true;
            m_referenceEcef = solution.ecefPosition;
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
    const bool res = solvePosition(ephemerides, measuredPseudoranges, transmitTimesSeconds, sol);

    outputSolution = solutionToOutput(sol);
    return res;
}
