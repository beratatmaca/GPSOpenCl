#ifndef INCLUDED_GPSOPENCL_ATMOSPHERICCORRECTIONS_H
#define INCLUDED_GPSOPENCL_ATMOSPHERICCORRECTIONS_H

#include "GPSOpenClPVTSolver.h"

#include <vector>

#include "GPSOpenClStructs.h"

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
    AtmosphericCorrections(const AtmosphericInput &input);
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

    static AtmosphericOutput computeCorrections(int svId,
                                                const GeodeticPosition &rxPos,
                                                const EcefPosition &rxEcef,
                                                const EcefPosition &satEcef,
                                                double gpsTimeSec,
                                                const AtmosphericInput &input);

    // Same as above, using the Klobuchar alpha/beta this instance was constructed with.
    AtmosphericOutput computeCorrections(int svId,
                                         const GeodeticPosition &rxPos,
                                         const EcefPosition &rxEcef,
                                         const EcefPosition &satEcef,
                                         double gpsTimeSec) const;

  private:
    AtmosphericInput m_inputConfig;
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_ATMOSPHERICCORRECTIONS_H
