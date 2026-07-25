#ifndef INCLUDED_GPSOPENCL_NAVIGATIONDECODER_H
#define INCLUDED_GPSOPENCL_NAVIGATIONDECODER_H

#include "GPSOpenClCommon.h"

#include <cstdint>
#include <vector>

namespace GPSOpenCl
{
struct GpsEphemeris
{
    int svId;
    int weekNumber;
    double tow; // Time of Week (seconds)
    int subframeId;
    bool isValid;

    // Clock correction parameters
    double toc; // Clock reference time (sec)
    double af0; // Clock bias (sec)
    double af1; // Clock drift (sec/sec)
    double af2; // Clock frequency drift (sec/sec^2)

    // Ephemeris orbit parameters
    double toe;      // Ephemeris reference time (sec)
    double sqrtA;    // Square root of semi-major axis (m^1/2)
    double e;        // Eccentricity
    double i0;       // Inclination angle at reference time (rad)
    double omega0;   // Longitude of ascending node (rad)
    double omega;    // Argument of perigee (rad)
    double M0;       // Mean anomaly at reference time (rad)
    double deltaN;   // Mean motion difference (rad/sec)
    double omegaDot; // Rate of right ascension (rad/sec)
    double idot;     // Rate of inclination angle (rad/sec)
    double Cuc, Cus, Crc, Crs, Cic, Cis; // Harmonic correction terms
};

class NavigationDecoder
{
  public:
    NavigationDecoder();
    ~NavigationDecoder();

    static bool findPreamble(const std::vector<bool> &bits, size_t &preambleIndex, bool &inverted);
    static bool checkParity(uint32_t word30bit, bool prevD29, bool prevD30);
    static int32_t extractSignedBits(uint32_t val, int startBit, int numBits);
    static uint32_t extractUnsignedBits(uint32_t val, int startBit, int numBits);

    bool decodeSubframe(const std::vector<uint32_t> &words30bit, GpsEphemeris &ephem);
    std::vector<bool> promptToBits(const ComplexFloatVector &promptHistory);

    // Searches promptHistory (from bitOffset onward) for the next subframe, verifies each word's
    // parity, applies the IS-GPS-200 D30* data-bit inversion, and decodes it. On success, advances
    // bitOffset past the consumed subframe (300 bits) so repeated calls with growing promptHistory
    // progress through subsequent subframes instead of re-finding the same one. On a parity failure,
    // advances bitOffset by 1 bit so the next call can resynchronize past the bad preamble candidate.
    // subframeStartSample is set to the promptHistory sample index where the decoded subframe began.
    bool processPromptSignal(int svId, const ComplexFloatVector &promptHistory, size_t &bitOffset,
                             GpsEphemeris &ephem, size_t &subframeStartSample);
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_NAVIGATIONDECODER_H
