#ifndef INCLUDED_GPSOPENCL_ATMOSPHERICCORRECTIONS_H
#define INCLUDED_GPSOPENCL_ATMOSPHERICCORRECTIONS_H

#include "GPSOpenClPVTSolver.h"

#include <vector>

namespace GPSOpenCl
{
struct KlobucharParams
{
    double alpha[4]; // alpha0..alpha3 (seconds)
    double beta[4];  // beta0..beta3 (seconds)
};

class AtmosphericCorrections
{
  public:
    AtmosphericCorrections();
    ~AtmosphericCorrections();

    static double klobucharIonosphericDelay(const GeodeticPosition &rxPos,
                                            double elevationDeg,
                                            double azimuthDeg,
                                            double gpsTimeSec,
                                            const KlobucharParams &params);

    static double saastamoinenTroposphericDelay(double rxAltitudeMeters,
                                               double elevationDeg);

    static void computeAzimuthElevation(const EcefPosition &rxEcef,
                                        const EcefPosition &satEcef,
                                        double &azimuthDeg,
                                        double &elevationDeg);
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_ATMOSPHERICCORRECTIONS_H
