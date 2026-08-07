#include "Pvt/GPSOpenClAtmosphericCorrections.hpp"

#include <algorithm>
#include <cmath>

using namespace GPSOpenCl;

AtmosphericOutput AtmosphericCorrections::computeCorrections(int svId,
                                                             const GeodeticPosition &rxPos,
                                                             const EcefPosition &rxEcef,
                                                             const EcefPosition &satEcef,
                                                             double gpsTimeSec,
                                                             const AtmosphericInput &input)
{
    AtmosphericOutput out{};
    out.structVersion = STRUCT_VERSION_1;
    out.svId = svId;

    double azDeg = 0.0;
    double elDeg = 0.0;
    computeAzimuthElevation(rxPos, rxEcef, satEcef, azDeg, elDeg);
    out.azimuthDeg = azDeg;
    out.elevationDeg = elDeg;

    KlobucharParams params{};
    params.alpha[0] = input.alpha0;
    params.alpha[1] = input.alpha1;
    params.alpha[2] = input.alpha2;
    params.alpha[3] = input.alpha3;
    params.beta[0] = input.beta0;
    params.beta[1] = input.beta1;
    params.beta[2] = input.beta2;
    params.beta[3] = input.beta3;

    out.ionoDelayMeters = klobucharIonosphericDelay(rxPos, elDeg, azDeg, gpsTimeSec, params);
    out.tropoDelayMeters = saastamoinenTroposphericDelay(rxPos.altitudeMeters, elDeg);

    return out;
}

void AtmosphericCorrections::computeAzimuthElevation(const EcefPosition &rxEcef,
                                                     const EcefPosition &satEcef,
                                                     double &azimuthDeg,
                                                     double &elevationDeg)
{
    computeAzimuthElevation(PVTSolver::ecefToWgs84(rxEcef), rxEcef, satEcef, azimuthDeg, elevationDeg);
}

void AtmosphericCorrections::computeAzimuthElevation(const GeodeticPosition &rxGeodeticPos,
                                                     const EcefPosition &rxEcef,
                                                     const EcefPosition &satEcef,
                                                     double &azimuthDeg,
                                                     double &elevationDeg)
{
    const double latRad = rxGeodeticPos.latitudeDeg * M_PI / 180.0;
    const double lonRad = rxGeodeticPos.longitudeDeg * M_PI / 180.0;

    const double dx = satEcef.x - rxEcef.x;
    const double dy = satEcef.y - rxEcef.y;
    const double dz = satEcef.z - rxEcef.z;

    const double sinLat = std::sin(latRad);
    const double cosLat = std::cos(latRad);
    const double sinLon = std::sin(lonRad);
    const double cosLon = std::cos(lonRad);

    const double east = (-sinLon * dx) + (cosLon * dy);
    const double north = (-sinLat * cosLon * dx) - (sinLat * sinLon * dy) + (cosLat * dz);
    const double up = (cosLat * cosLon * dx) + (cosLat * sinLon * dy) + (sinLat * dz);

    const double horizontalDist = std::sqrt((east * east) + (north * north));
    elevationDeg = std::atan2(up, horizontalDist) * 180.0 / M_PI;
    double azRad = std::atan2(east, north);
    if (azRad < 0.0)
    {
        azRad += 2.0 * M_PI;
    }
    azimuthDeg = azRad * 180.0 / M_PI;
}

double AtmosphericCorrections::klobucharIonosphericDelay(const GeodeticPosition &rxPos,
                                                         double elevationDeg,
                                                         double azimuthDeg,
                                                         double gpsTimeSec,
                                                         const KlobucharParams &params)
{

    const double phiU = rxPos.latitudeDeg / 180.0;
    const double lambdaU = rxPos.longitudeDeg / 180.0;
    double el = elevationDeg / 180.0;
    const double az = azimuthDeg * M_PI / 180.0;

    el = std::max(el, 0.0);

    const double psi = (0.0137 / (el + 0.11)) - 0.022;

    double phiI = phiU + (psi * std::cos(az));
    phiI = std::clamp(phiI, -0.416, 0.416);

    const double lambdaI = lambdaU + ((psi * std::sin(az)) / std::cos(phiI * M_PI));

    const double phiM = phiI + (0.064 * std::cos((lambdaI - 1.617) * M_PI));

    double tLocal = (43200.0 * lambdaI) + gpsTimeSec;
    tLocal = std::fmod(tLocal, 86400.0);
    if (tLocal < 0.0)
    {
        tLocal += 86400.0;
    }

    const double F = 1.0 + (16.0 * std::pow(0.53 - el, 3));

    double per = params.beta[0] + (params.beta[1] * phiM) + (params.beta[2] * phiM * phiM) +
        (params.beta[3] * phiM * phiM * phiM);
    per = std::max(per, 72000.0);

    double amp = params.alpha[0] + (params.alpha[1] * phiM) + (params.alpha[2] * phiM * phiM) +
        (params.alpha[3] * phiM * phiM * phiM);
    amp = std::max(amp, 0.0);

    const double x = 2.0 * M_PI * (tLocal - 50400.0) / per;

    double delaySec = 0.0;
    if (std::fabs(x) < 1.57)
    {
        delaySec = F * (5e-9 + amp * (1.0 - (x * x) / 2.0 + (x * x * x * x) / 24.0));
    }
    else
    {
        delaySec = F * 5e-9;
    }

    return delaySec * SPEED_OF_LIGHT_M_S;
}

double AtmosphericCorrections::saastamoinenTroposphericDelay(double rxAltitudeMeters, double elevationDeg)
{
    elevationDeg = std::max(elevationDeg, 2.0);

    double h = rxAltitudeMeters;
    h = std::max(h, 0.0);
    h = std::min(h, 10000.0);

    const double p0 = 1013.25;
    const double T0 = 288.15;
    const double e0 = 11.691;

    const double p = p0 * std::pow(1.0 - (2.2557e-5 * h), 5.2568);
    const double T = T0 - (0.0065 * h);
    const double e = e0 * std::pow(1.0 - (2.2557e-5 * h), 11.96);

    const double elRad = elevationDeg * M_PI / 180.0;
    const double tanEl = std::tan(elRad);

    const double mappedEl = std::sin(elRad + (0.00143 / (tanEl + 0.0445)));

    const double dryDelay = (0.002277 * p) / mappedEl;
    const double wetDelay = (0.002277 * (1255.0 / T + 0.05) * e) / mappedEl;

    return dryDelay + wetDelay;
}
