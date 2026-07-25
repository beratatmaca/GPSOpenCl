#ifndef INCLUDED_GPSOPENCL_PVTSOLVER_H
#define INCLUDED_GPSOPENCL_PVTSOLVER_H

#include "GPSOpenClCommon.h"
#include "GPSOpenClNavigationDecoder.h"

#include <vector>

namespace GPSOpenCl
{
struct EcefPosition
{
    double x; // meters
    double y; // meters
    double z; // meters
};

struct GeodeticPosition
{
    double latitude;  // degrees (-90 to +90)
    double longitude; // degrees (-180 to +180)
    double altitude;  // meters above WGS84 ellipsoid
};

struct SatelliteOrbit
{
    int svId;
    EcefPosition position;
    double clockBias; // seconds
    double relCorr;   // seconds
};

struct ReceiverPvtSolution
{
    EcefPosition ecefPosition;
    GeodeticPosition geodeticPosition;
    double clockBiasMeters;
    double clockBiasSeconds;
    double dopGDOP;
    double dopPDOP;
    double dopHDOP;
    double dopVDOP;
    bool isValid;
};

class PVTSolver
{
  public:
    PVTSolver();
    ~PVTSolver();

    static SatelliteOrbit computeSatelliteOrbit(const GpsEphemeris &ephem, double transmitTimeSeconds);
    static GeodeticPosition ecefToWgs84(const EcefPosition &ecef);

    bool solvePosition(const std::vector<GpsEphemeris> &ephemerides,
                       const std::vector<double> &measuredPseudoranges,
                       const std::vector<double> &transmitTimesSeconds,
                       ReceiverPvtSolution &solution);
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_PVTSOLVER_H
