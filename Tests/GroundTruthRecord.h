#ifndef INCLUDED_GPSOPENCLTEST_GROUNDTRUTHRECORD_H
#define INCLUDED_GPSOPENCLTEST_GROUNDTRUTHRECORD_H

/** @file GroundTruthRecord.h
 *  @brief C++ mirror of gps-sdr-sim's -G ground-truth export record.
 */

#include <cstdint>

namespace GPSOpenClTest
{
#pragma pack(push, 1)

/** @brief One simulator-truth record for one satellite at one RF-sample boundary (~1ms spacing).
 *
 *  @a sampleIndex is the 0-based index of the IQ sample pair in the RF output stream that
 *  trueDopplerHz/trueCodePhaseChips/trueDataBit are exact for -- the authoritative key for
 *  correlating a record with an exact RF sample. The remaining fields (range, rate, geometry,
 *  satellite/receiver position, clock) hold their value from the simulator's last 0.1s geometry
 *  update and are exact for that update, not interpolated between updates. */
struct GroundTruthRecord
{
    uint32_t structVersion;           ///< Record format version, must be 2.
    uint32_t chunkIndex;              ///< iumd counter (0.1s outer-loop index); coarse bucketing only.
    uint64_t sampleIndex;             ///< Absolute 0-based IQ sample index in the RF output stream.
    double gpsTimeSec;                ///< GPS time-of-week (s) of this exact sample.
    int32_t gpsWeek;                  ///< GPS week number.
    int32_t prn;                      ///< Satellite PRN number.
    double trueDopplerHz;             ///< True carrier Doppler (Hz), same sign convention as Tracking::m_carrFreqHz.
    double trueCodePhaseChips;        ///< True code phase (chips, 0..1023), exact at sampleIndex.
    int32_t trueDataBit;              ///< True navigation data bit (+-1), exact at sampleIndex.
    double truePseudorangeM;          ///< True pseudorange (m), includes ionospheric delay.
    double trueGeometricRangeM;       ///< True geometric range (m), no clock/iono error.
    double truePseudorangeRateMps;    ///< True pseudorange rate (m/s).
    double trueIonoDelayM;            ///< True ionospheric delay (m).
    double trueAzimuthDeg;            ///< True satellite azimuth (deg).
    double trueElevationDeg;          ///< True satellite elevation (deg).
    double trueSatPosXEcefM;          ///< Satellite ECEF position at transmit time (m).
    double trueSatPosYEcefM;          ///< Satellite ECEF position at transmit time (m).
    double trueSatPosZEcefM;          ///< Satellite ECEF position at transmit time (m).
    double trueSatVelXEcefMps;        ///< Satellite ECEF velocity at transmit time (m/s).
    double trueSatVelYEcefMps;        ///< Satellite ECEF velocity at transmit time (m/s).
    double trueSatVelZEcefMps;        ///< Satellite ECEF velocity at transmit time (m/s).
    double trueSvClockBiasSec;        ///< Satellite clock bias (s).
    double trueSvClockDriftSec;       ///< Satellite clock drift (s/s).
    double trueReceiverPosXEcefM;     ///< Receiver ECEF position for this update (m).
    double trueReceiverPosYEcefM;     ///< Receiver ECEF position for this update (m).
    double trueReceiverPosZEcefM;     ///< Receiver ECEF position for this update (m).
};

#pragma pack(pop)
}    // namespace GPSOpenClTest

#endif    //! INCLUDED_GPSOPENCLTEST_GROUNDTRUTHRECORD_H
