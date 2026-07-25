#ifndef INCLUDED_GPSOPENCL_PVTSOLVER_H
#define INCLUDED_GPSOPENCL_PVTSOLVER_H

#include "GPSOpenClCommon.h"
#include "GPSOpenClNavigationDecoder.h"

#include <vector>

#include "GPSOpenClStructs.h"

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
    PVTSolver(const PvtSolverInput &input);
    ~PVTSolver();

    static SatelliteOrbit computeSatelliteOrbit(const GpsEphemeris &ephem, double transmitTimeSeconds);
    static SatelliteOrbit computeSatelliteOrbit(const NavDecoderOutput &navOut, double transmitTimeSeconds);
    static GeodeticPosition ecefToWgs84(const EcefPosition &ecef);

    static PvtSolverOutput solutionToOutput(const ReceiverPvtSolution &sol);
    static ReceiverPvtSolution outputToSolution(const PvtSolverOutput &out);

    bool solvePosition(const std::vector<GpsEphemeris> &ephemerides,
                       const std::vector<double> &measuredPseudoranges,
                       const std::vector<double> &transmitTimesSeconds,
                       ReceiverPvtSolution &solution);

    bool solvePosition(const std::vector<NavDecoderOutput> &outputs,
                       const std::vector<double> &measuredPseudoranges,
                       const std::vector<double> &transmitTimesSeconds,
                       PvtSolverOutput &outputSolution);

  private:
    PvtSolverInput m_inputConfig;
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_PVTSOLVER_H
