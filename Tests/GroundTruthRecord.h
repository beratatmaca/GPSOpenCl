#ifndef INCLUDED_GPSOPENCLTEST_GROUNDTRUTHRECORD_H
#define INCLUDED_GPSOPENCLTEST_GROUNDTRUTHRECORD_H

/** @file GroundTruthRecord.h
 *  @brief C++ mirror of gps-sdr-sim's -G ground-truth export record.
 */

#include <cstdint>

namespace GPSOpenClTest
{
#pragma pack(push, 1)

/** @brief One simulator-truth record for one channel at one 0.1s epoch. */
struct GroundTruthRecord
{
    uint32_t structVersion;       ///< Record format version, must be 1.
    uint32_t chunkIndex;          ///< iumd counter, diagnostic only, not sample-exact.
    double gpsTimeSec;            ///< GPS time-of-week (s), authoritative time reference.
    int32_t prn;                  ///< Satellite PRN number.
    double trueDopplerHz;         ///< True carrier Doppler (Hz), same sign convention as Tracking::m_carrFreq.
    double trueCodePhaseChips;    ///< True code phase (chips, 0..1023).
    int32_t trueDataBit;          ///< True navigation data bit (+-1).
    double truePseudorangeM;      ///< True pseudorange (m).
    double trueGeometricRangeM;   ///< True geometric range (m), no clock/iono error.
    double trueAzimuthDeg;        ///< True satellite azimuth (deg).
    double trueElevationDeg;      ///< True satellite elevation (deg).
};

#pragma pack(pop)
} // namespace GPSOpenClTest

#endif //! INCLUDED_GPSOPENCLTEST_GROUNDTRUTHRECORD_H
