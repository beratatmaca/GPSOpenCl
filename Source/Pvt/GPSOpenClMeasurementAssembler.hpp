#ifndef INCLUDED_GPSOPENCL_MEASUREMENTASSEMBLER_HPP
#define INCLUDED_GPSOPENCL_MEASUREMENTASSEMBLER_HPP

/** @file GPSOpenClMeasurementAssembler.hpp
 *  @brief Builds pseudorange measurements from channel state.
 */

#include "NavDecode/GPSOpenClNavigationDecoder.hpp"
#include "Pvt/GPSOpenClPVTSolver.hpp"
#include "Tracking/GPSOpenClChannel.hpp"

#include <vector>

namespace GPSOpenCl
{
/** @brief Turns tracked channel state into PVT solver inputs.
 *   Internal pipeline helper. Reconstructs each satellite transmit
 *   time from its subframe anchor. Resolves whole millisecond
 *   ambiguities against the constellation. Forms pseudoranges. */
class MeasurementAssembler
{
  public:
    /** @brief One epoch of solver ready measurements. */
    struct Measurements
    {
        std::vector<GpsEphemeris> ephemerides;     ///< One ephemeris per usable satellite.
        std::vector<double> transmitTimesSec;      ///< Reconstructed transmit times in seconds.
        std::vector<double> pseudorangesMeters;    ///< Measured pseudoranges in meters.
        std::vector<int> prns;                     ///< Satellite PRNs, same order.
        double receiverTimeSec{0.0};               ///< Common receiver time estimate in seconds.
    };

    /** @brief Compute whole code periods elapsed between a subframe anchor and the buffer end.
     *  @param promptCount       Prompt history length in code periods.
     *  @param startSample       Anchor block index within the prompt history.
     *  @param elapsedSecondsOut Elapsed time since the anchor in seconds (output).
     *  @return False when the anchor lies beyond the buffer. */
    static bool computeElapsedSecondsSincePromptStart(size_t promptCount, size_t startSample, double &elapsedSecondsOut);

    /** @brief Compute satellite transmit time from the subframe anchor.
     *   Adds a sub millisecond code phase term. Without it every
     *   pseudorange quantizes to the 1 ms block grid, up to 300 km error.
     *  @param subframeStartTow TOW at the subframe leading bit edge in seconds.
     *  @param elapsedSeconds   Whole code periods since the anchor block in seconds.
     *  @param driftChips       Accumulated code frequency drift since the anchor in chips.
     *  @param anchorChipsRaw   DLL code phase at the anchor block in chips, 0 to 1023.
     *  @return Satellite transmit time in seconds of week. */
    static double computeTransmitTime(double subframeStartTow, double elapsedSeconds, double driftChips, double anchorChipsRaw);

    /** @brief Resolve C A millisecond ambiguity across the constellation.
     *   A bit edge attributed one block off shifts a transmit time by a
     *   whole millisecond. Snaps such outliers onto the median cluster.
     *   The cluster need not hold the true epoch. A common shift is
     *   absorbed by the receiver clock bias, so only consistency matters.
     *   Corrections are limited to one code period: the bit edge
     *   ambiguity is inherently plus or minus one block, so any larger
     *   implied offset means the reference geometry is wrong and the
     *   satellite is left untouched for the solver's residual gate.
     *  @param transmitTimes   Per satellite transmit times in seconds, corrected in place.
     *  @param impliedArrivals Per satellite modeled arrival instants in seconds, same order. */
    static void snapTransmitTimesToMedianArrival(std::vector<double> &transmitTimes, const std::vector<double> &impliedArrivals);

    /** @brief Gather measurements from every usable channel.
     *   A channel needs confirmed tracking, a complete ephemeris, and a
     *   fresh subframe anchor. Needs at least four satellites.
     *   Millisecond snapping runs only with a trusted reference. An
     *   unfixed seed far from the true position makes differential
     *   transit errors exceed half a code period and the snap would
     *   corrupt transmit times by whole milliseconds.
     *  @param channels         Channel array.
     *  @param channelCount     Number of channels.
     *  @param referenceEcef    Coarse receiver position for transit time modeling.
     *  @param referenceTrusted True once the reference comes from a successful fix.
     *  @param out              Assembled measurements (output).
     *  @return True if at least four satellites were usable. */
    static bool assemble(const Channel *channels, int channelCount, const EcefPosition &referenceEcef, bool referenceTrusted, Measurements &out);
};
}

#endif
