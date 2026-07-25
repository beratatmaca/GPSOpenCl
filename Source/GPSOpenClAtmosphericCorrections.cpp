#include "GPSOpenClAtmosphericCorrections.h"

#include <algorithm>
#include <cmath>

using namespace GPSOpenCl;

AtmosphericCorrections::AtmosphericCorrections()
    : m_inputConfig{STRUCT_VERSION_1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
{
}

AtmosphericCorrections::AtmosphericCorrections(const AtmosphericInput &input)
    : m_inputConfig(input)
{
}

AtmosphericCorrections::~AtmosphericCorrections()
{
}

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

    double azDeg = 0.0, elDeg = 0.0;
    computeAzimuthElevation(rxEcef, satEcef, azDeg, elDeg);
    out.azimuthDeg = azDeg;
    out.elevationDeg = elDeg;

    KlobucharParams params;
    params.alpha[0] = input.alpha0;
    params.alpha[1] = input.alpha1;
    params.alpha[2] = input.alpha2;
    params.alpha[3] = input.alpha3;
    params.beta[0] = input.beta0;
    params.beta[1] = input.beta1;
    params.beta[2] = input.beta2;
    params.beta[3] = input.beta3;

    double ionoDelaySec = klobucharIonosphericDelay(rxPos, elDeg, azDeg, gpsTimeSec, params);
    const double c = 299792458.0;
    out.ionoDelayMeters = ionoDelaySec * c;
    out.tropoDelayMeters = saastamoinenTroposphericDelay(rxPos.altitude, elDeg);

    return out;
}

AtmosphericOutput AtmosphericCorrections::computeCorrections(int svId,
                                                              const GeodeticPosition &rxPos,
                                                              const EcefPosition &rxEcef,
                                                              const EcefPosition &satEcef,
                                                              double gpsTimeSec) const
{
    return computeCorrections(svId, rxPos, rxEcef, satEcef, gpsTimeSec, m_inputConfig);
}

void AtmosphericCorrections::computeAzimuthElevation(const EcefPosition &rxEcef,
                                                     const EcefPosition &satEcef,
                                                     double &azimuthDeg,
                                                     double &elevationDeg)
{
    GeodeticPosition geo = PVTSolver::ecefToWgs84(rxEcef);
    double latRad = geo.latitude * M_PI / 180.0;
    double lonRad = geo.longitude * M_PI / 180.0;

    double dx = satEcef.x - rxEcef.x;
    double dy = satEcef.y - rxEcef.y;
    double dz = satEcef.z - rxEcef.z;


    double sinLat = std::sin(latRad);
    double cosLat = std::cos(latRad);
    double sinLon = std::sin(lonRad);
    double cosLon = std::cos(lonRad);

    double east  = -sinLon * dx + cosLon * dy;
    double north = -sinLat * cosLon * dx - sinLat * sinLon * dy + cosLat * dz;
    double up    =  cosLat * cosLon * dx + cosLat * sinLon * dy + sinLat * dz;

    double horizontalDist = std::sqrt(east * east + north * north);
    elevationDeg = std::atan2(up, horizontalDist) * 180.0 / M_PI;
    double azRad = std::atan2(east, north);
    if (azRad < 0.0) azRad += 2.0 * M_PI;
    azimuthDeg = azRad * 180.0 / M_PI;
}

double AtmosphericCorrections::klobucharIonosphericDelay(const GeodeticPosition &rxPos,
                                                         double elevationDeg,
                                                         double azimuthDeg,
                                                         double gpsTimeSec,
                                                         const KlobucharParams &params)
{
    const double c = 299792458.0;


    double phiU = rxPos.latitude / 180.0;
    double lambdaU = rxPos.longitude / 180.0;
    double el = elevationDeg / 180.0;
    double az = azimuthDeg * M_PI / 180.0;

    if (el < 0.0) el = 0.0;


    double psi = 0.0137 / (el + 0.11) - 0.022;


    double phiI = phiU + psi * std::cos(az);
    phiI = std::clamp(phiI, -0.416, 0.416);


    double lambdaI = lambdaU + (psi * std::sin(az)) / std::cos(phiI * M_PI);


    double phiM = phiI + 0.064 * std::cos((lambdaI - 1.617) * M_PI);


    double tLocal = 43200.0 * lambdaI + gpsTimeSec;
    tLocal = std::fmod(tLocal, 86400.0);
    if (tLocal < 0.0) tLocal += 86400.0;


    double F = 1.0 + 16.0 * std::pow(0.53 - el, 3);


    double PER = params.beta[0] + params.beta[1] * phiM +
                 params.beta[2] * phiM * phiM + params.beta[3] * phiM * phiM * phiM;
    if (PER < 72000.0) PER = 72000.0;


    double AMP = params.alpha[0] + params.alpha[1] * phiM +
                 params.alpha[2] * phiM * phiM + params.alpha[3] * phiM * phiM * phiM;
    if (AMP < 0.0) AMP = 0.0;


    double x = 2.0 * M_PI * (tLocal - 50400.0) / PER;

    double delaySec = 0.0;
    if (std::fabs(x) < 1.57)
    {
        delaySec = F * (5e-9 + AMP * (1.0 - (x * x) / 2.0 + (x * x * x * x) / 24.0));
    }
    else
    {
        delaySec = F * 5e-9;
    }

    return delaySec * c;
}

double AtmosphericCorrections::saastamoinenTroposphericDelay(double rxAltitudeMeters, double elevationDeg)
{
    if (elevationDeg < 2.0) elevationDeg = 2.0;

    double h = rxAltitudeMeters;
    if (h < 0.0) h = 0.0;


    double p0 = 1013.25;
    double T0 = 288.15;
    double e0 = 11.691;

    double p = p0 * std::pow(1.0 - 2.2557e-5 * h, 5.2568);
    double T = T0 - 0.0065 * h;
    double e = e0 * std::pow(1.0 - 2.2557e-5 * h, 11.96);

    double elRad = elevationDeg * M_PI / 180.0;
    double tanEl = std::tan(elRad);

    double mappedEl = std::sin(elRad + 0.00143 / (tanEl + 0.045));

    double dryDelay = (0.002277 * p) / mappedEl;
    double wetDelay = (0.002277 * (1255.0 / T + 0.05) * e) / mappedEl;

    return dryDelay + wetDelay;
}
