#ifndef INCLUDED_GPSOPENCL_ATMOSPHERICCORRECTIONS_H
#define INCLUDED_GPSOPENCL_ATMOSPHERICCORRECTIONS_H

/** @file GPSOpenClAtmosphericCorrections.h
 *  @brief Ionospheric and tropospheric delay corrections.
 */

#include "GPSOpenClPVTSolver.h"

#include <vector>

#include "GPSOpenClStructs.h"

namespace GPSOpenCl
{
/** @brief Klobuchar ionospheric model coefficients. */
struct KlobucharParams
{
    double alpha[4]; ///< Alpha coefficients (s).
    double beta[4];  ///< Beta coefficients (s).
};

/** @brief Atmospheric delay correction calculator. */
class AtmosphericCorrections
{
  public:
    AtmosphericCorrections();

    /** @brief Construct from atmospheric parameters.
     *  @param input Klobuchar alpha/beta coefficients. */
    AtmosphericCorrections(const AtmosphericInput &input);

    ~AtmosphericCorrections();

    /** @brief Compute Klobuchar ionospheric delay.
     *  @param rxPos        Receiver geodetic position.
     *  @param elevationDeg Satellite elevation (deg).
     *  @param azimuthDeg   Satellite azimuth (deg).
     *  @param gpsTimeSec   GPS time of week (s).
     *  @param params       Klobuchar alpha/beta coefficients.
     *  @return Ionospheric delay (m). */
    static double klobucharIonosphericDelay(const GeodeticPosition &rxPos,
                                            double elevationDeg,
                                            double azimuthDeg,
                                            double gpsTimeSec,
                                            const KlobucharParams &params);

    /** @brief Compute Saastamoinen tropospheric delay.
     *  @param rxAltitudeMeters Receiver altitude (m).
     *  @param elevationDeg     Satellite elevation (deg).
     *  @return Tropospheric delay (m). */
    static double saastamoinenTroposphericDelay(double rxAltitudeMeters,
                                               double elevationDeg);

    /** @brief Compute azimuth and elevation from receiver to satellite.
     *  @param rxEcef       Receiver ECEF position.
     *  @param satEcef      Satellite ECEF position.
     *  @param azimuthDeg   Azimuth output (deg).
     *  @param elevationDeg Elevation output (deg). */
    static void computeAzimuthElevation(const EcefPosition &rxEcef,
                                        const EcefPosition &satEcef,
                                        double &azimuthDeg,
                                        double &elevationDeg);

    /** @brief Compute all atmospheric corrections for one satellite (static).
     *  @param svId       Satellite vehicle ID.
     *  @param rxPos      Receiver geodetic position.
     *  @param rxEcef     Receiver ECEF position.
     *  @param satEcef    Satellite ECEF position.
     *  @param gpsTimeSec GPS time of week (s).
     *  @param input      Klobuchar alpha/beta coefficients.
     *  @return Atmospheric correction output. */
    static AtmosphericOutput computeCorrections(int svId,
                                                const GeodeticPosition &rxPos,
                                                const EcefPosition &rxEcef,
                                                const EcefPosition &satEcef,
                                                double gpsTimeSec,
                                                const AtmosphericInput &input);

    /** @brief Compute corrections using instance Klobuchar parameters.
     *  @param svId       Satellite vehicle ID.
     *  @param rxPos      Receiver geodetic position.
     *  @param rxEcef     Receiver ECEF position.
     *  @param satEcef    Satellite ECEF position.
     *  @param gpsTimeSec GPS time of week (s).
     *  @return Atmospheric correction output. */
    AtmosphericOutput computeCorrections(int svId,
                                         const GeodeticPosition &rxPos,
                                         const EcefPosition &rxEcef,
                                         const EcefPosition &satEcef,
                                         double gpsTimeSec) const;

  private:
    AtmosphericInput m_inputConfig; ///< Klobuchar coefficients.
};
}

#endif
