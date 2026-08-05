#ifndef INCLUDED_GPSOPENCL_PVTSOLVER_H
#define INCLUDED_GPSOPENCL_PVTSOLVER_H

/** @file GPSOpenClPVTSolver.h
 *  @brief WLS position solver with Keplerian orbit computation.
 */

#include "GPSOpenClCommon.h"
#include "GPSOpenClNavigationDecoder.h"

#include <vector>

#include "GPSOpenClStructs.h"

namespace GPSOpenCl
{
/** @brief ECEF position in meters (WGS-84). */
struct EcefPosition
{
    double x;    ///< X coordinate (m).
    double y;    ///< Y coordinate (m).
    double z;    ///< Z coordinate (m).
};

/** @brief Geodetic position (WGS-84). */
struct GeodeticPosition
{
    double latitude;     ///< Latitude (deg, -90 to +90).
    double longitude;    ///< Longitude (deg, -180 to +180).
    double altitude;     ///< Altitude above WGS-84 ellipsoid (m).
};

/** @brief Satellite ECEF position and clock correction. */
struct SatelliteOrbit
{
    int svId;                 ///< Satellite vehicle ID.
    EcefPosition position;    ///< ECEF position (m).
    double clockBias;         ///< Clock bias (s).
    double relCorr;           ///< Relativistic correction (s).
};

/** @brief Receiver position solution with DOP values. */
struct ReceiverPvtSolution
{
    EcefPosition ecefPosition;            ///< ECEF position (m).
    GeodeticPosition geodeticPosition;    ///< Geodetic lat/lon/alt.
    double clockBiasMeters;               ///< Receiver clock bias (m).
    double clockBiasSeconds;              ///< Receiver clock bias (s).
    double dopGDOP;                       ///< Geometric DOP.
    double dopPDOP;                       ///< Position DOP.
    double dopHDOP;                       ///< Horizontal DOP.
    double dopVDOP;                       ///< Vertical DOP.
    bool isValid;                         ///< True if solution valid.
};

/** @brief Position-Velocity-Time solver. */
class PVTSolver
{
  public:
    PVTSolver();

    /** @brief Construct from solver parameters.
     *  @param input Solver settings. */
    PVTSolver(const PvtSolverInput &input);

    ~PVTSolver() = default;
    PVTSolver(const PVTSolver &) = delete;
    PVTSolver &operator=(const PVTSolver &) = delete;
    PVTSolver(PVTSolver &&) = delete;
    PVTSolver &operator=(PVTSolver &&) = delete;

    /** @brief Compute satellite ECEF position at transmit time.
     *  @param ephem              Decoded ephemeris.
     *  @param transmitTimeSeconds GPS transmit time (s).
     *  @return Satellite orbit and clock correction. */
    static SatelliteOrbit computeSatelliteOrbit(const GpsEphemeris &ephem, double transmitTimeSeconds);

    /** @brief Compute satellite ECEF position from NavDecoderOutput.
     *  @param navOut             Nav decoder output struct.
     *  @param transmitTimeSeconds GPS transmit time (s).
     *  @return Satellite orbit and clock correction. */
    static SatelliteOrbit computeSatelliteOrbit(const NavDecoderOutput &navOut, double transmitTimeSeconds);

    /** @brief Convert ECEF to WGS-84 geodetic coordinates.
     *  @param ecef ECEF position (m).
     *  @return Geodetic lat/lon/alt. */
    static GeodeticPosition ecefToWgs84(const EcefPosition &ecef);

    /** @brief Estimate the common receiver time before an absolute receiver clock is known: the
     *   average of each satellite's transmit time plus the average geometric transit time from a
     *   reference position to that satellite (evaluated at that satellite's own transmit time).
     *   Exact (equals the true receive instant) if referenceEcef is the true receiver position and
     *   every transmitTimeSeconds[i] is the true transmit time of the signal currently arriving from
     *   satellite i -- then transmitTimeSeconds[i] + transitTime[i] is the same true instant for
     *   every i, so the two averages sum back to it regardless of per-satellite geometry.
     *  @param ephemerides          Satellite ephemeris data.
     *  @param transmitTimesSeconds Signal transmit times (s), one per satellite.
     *  @param referenceEcef        Reference receiver position (m).
     *  @return Receiver time estimate (s), or 0.0 if transmitTimesSeconds is empty. */
    static double computeReceiverTime(const std::vector<GpsEphemeris> &ephemerides,
                                      const std::vector<double> &transmitTimesSeconds,
                                      const EcefPosition &referenceEcef);

    /** @brief Convert ReceiverPvtSolution to PvtSolverOutput.
     *  @param sol Solution struct.
     *  @return Output struct. */
    static PvtSolverOutput solutionToOutput(const ReceiverPvtSolution &sol);

    /** @brief Convert PvtSolverOutput to ReceiverPvtSolution.
     *  @param out Output struct.
     *  @return Solution struct. */
    static ReceiverPvtSolution outputToSolution(const PvtSolverOutput &out);

    /** @brief Solve receiver position using Weighted Least Squares. Each satellite is weighted by
     *   sin^2(elevation), so low-elevation satellites (noisier, more multipath/atmospheric error)
     *   contribute less to the solution than high-elevation ones. m_referenceEcef seeds both the
     *   caller's transit-time estimate and this solve's initial state, and is updated from the
     *   converged state on every call -- including one rejected for exceeding
     *   maxPseudorangeErrMeters -- so a coarse or wrong seed self-corrects across repeated calls
     *   instead of permanently rejecting every fix.
     *  @param ephemerides          Satellite ephemeris data.
     *  @param measuredPseudoranges Pseudorange measurements (m).
     *  @param transmitTimesSeconds Signal transmit times (s).
     *  @param solution             Output position solution.
     *  @return True if solution converged. */
    bool solvePosition(const std::vector<GpsEphemeris> &ephemerides,
                       const std::vector<double> &measuredPseudoranges,
                       const std::vector<double> &transmitTimesSeconds,
                       ReceiverPvtSolution &solution);

    /** @brief Solve receiver position from NavDecoderOutput structs.
     *  @param outputs              Nav decoder outputs.
     *  @param measuredPseudoranges Pseudorange measurements (m).
     *  @param transmitTimesSeconds Signal transmit times (s).
     *  @param outputSolution       Output struct.
     *  @return True if solution converged. */
    bool solvePosition(const std::vector<NavDecoderOutput> &outputs,
                       const std::vector<double> &measuredPseudoranges,
                       const std::vector<double> &transmitTimesSeconds,
                       PvtSolverOutput &outputSolution);

    /** @brief Set broadcast Klobuchar ionospheric coefficients used for atmospheric correction.
     *  @param params Alpha/beta coefficients (e.g. from NavigationDecoder::getIonosphericParams). */
    void setIonosphericParams(const AtmosphericInput &params) { m_ionoParams = params; }

    /** @brief Get the current coarse receiver position estimate, used to seed the Newton-Raphson
     *   solve and to derive per-satellite transit-time estimates before a fix has converged.
     *   Updated to the converged position after every successful solvePosition() call.
     *  @return Best-known ECEF receiver position (m). */
    EcefPosition getReferenceEcef() const { return m_referenceEcef; }

  private:
    PvtSolverInput m_inputConfig;       ///< Solver parameters.
    AtmosphericInput m_ionoParams{};    ///< Broadcast Klobuchar coefficients (zero until decoded).
    EcefPosition m_referenceEcef{
        4180483.4,
        851798.0,
        4725999.8};    ///< Coarse receiver position estimate (m), seeds the solve and transit-time estimates.
    bool m_hasValidFix{false};    ///< True once a solve has succeeded; gates the elevation mask.
};
}

#endif
