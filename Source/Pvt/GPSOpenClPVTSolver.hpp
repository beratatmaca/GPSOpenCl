#ifndef INCLUDED_GPSOPENCL_PVTSOLVER_HPP
#define INCLUDED_GPSOPENCL_PVTSOLVER_HPP

/** @file GPSOpenClPVTSolver.hpp
 *  @brief WLS position solver with Keplerian orbit computation.
 */

#include "Common/GPSOpenClCommon.hpp"
#include "NavDecode/GPSOpenClNavigationDecoder.hpp"

#include <vector>

#include "Common/GPSOpenClStructs.hpp"

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
    double latitudeDeg;       ///< Latitude in degrees, -90 to +90.
    double longitudeDeg;      ///< Longitude in degrees, -180 to +180.
    double altitudeMeters;    ///< Altitude above WGS-84 ellipsoid in meters.
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
    uint32_t satellitesUsed;              ///< Satellites in the accepted solution.
    double maxResidualMeters;             ///< Largest converged pseudorange residual (m).
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

    /** @brief Convert ECEF to WGS-84 geodetic coordinates.
     *  @param ecef ECEF position (m).
     *  @return Geodetic lat/lon/alt. */
    static GeodeticPosition ecefToWgs84(const EcefPosition &ecef);

    /** @brief Estimate the common receiver time before a fix. Averages transmit time plus geometric
     *   transit time. Transit time comes from the reference position. Exact when position and
     *   transmit times are true. Then transmit plus transit is one shared instant. Averaging gives
     *   it back regardless of geometry.
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

    /** @brief Solve receiver position with weighted least squares. Weights are sin^2 of elevation.
     *   Low satellites carry more noise and multipath. The elevation mask needs a trusted position.
     *   So it engages only after a successful solve. A worst residual above the gate drops that
     *   satellite. The solve then repeats with the rest. One faulted measurement cannot poison the
     *   fix. m_referenceEcef seeds the solve and transit times. It updates only after a
     *   successful solve, so it stays at the compiled-in seed until the first fix.
     *  @param ephemerides          Satellite ephemeris data.
     *  @param measuredPseudoranges Pseudorange measurements (m).
     *  @param transmitTimesSeconds Signal transmit times (s).
     *  @param solution             Output position solution.
     *  @return True if solution converged. */
    bool solvePosition(const std::vector<GpsEphemeris> &ephemerides,
                       const std::vector<double> &measuredPseudoranges,
                       const std::vector<double> &transmitTimesSeconds,
                       ReceiverPvtSolution &solution);

    /** @brief Set broadcast Klobuchar ionospheric coefficients used for atmospheric correction.
     *  @param params Alpha and beta coefficients, e.g. from the nav decoder. */
    void setIonosphericParams(const AtmosphericInput &params) { m_ionoParams = params; }

    /** @brief Get the coarse receiver position estimate. Seeds the solve and transit time
     *   estimates. Updated after every successful solve.
     *  @return Best known ECEF receiver position (m). */
    EcefPosition getReferenceEcef() const { return m_referenceEcef; }

    /** @brief Whether a solve has ever succeeded, i.e. the reference position is trustworthy.
     *  @return True after the first successful fix. */
    bool hasValidFix() const { return m_hasValidFix; }

  private:
    PvtSolverInput m_inputConfig;       ///< Solver parameters.
    AtmosphericInput m_ionoParams{};    ///< Broadcast Klobuchar coefficients (zero until decoded).
    EcefPosition m_referenceEcef{
        4180483.4,
        851798.0,
        4725999.8};               ///< Coarse receiver position estimate (m), seeds the solve and transit times.
    bool m_hasValidFix{false};    ///< True once a solve has succeeded, gates the elevation mask.
};
}

#endif
