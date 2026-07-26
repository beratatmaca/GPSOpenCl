#ifndef INCLUDED_GPSOPENCL_LOCKDETECTOR_H
#define INCLUDED_GPSOPENCL_LOCKDETECTOR_H

/** @file GPSOpenClLockDetector.h
 *  @brief Carrier and code lock quality indicators.
 */

namespace GPSOpenCl
{
/** @brief Stateless carrier/code lock indicator calculations. */
class LockDetector
{
  public:
    /** @brief Normalized narrowband carrier lock indicator.
     *  @param Ip In-phase prompt correlator sum.
     *  @param Qp Quadrature prompt correlator sum.
     *  @return Value near +1 when phase-locked, near 0 when unlocked. */
    static double carrierLockIndicator(double Ip, double Qp);

    /** @brief Normalized early/late code lock ratio.
     *  @param Ie In-phase early correlator sum.
     *  @param Ip In-phase prompt correlator sum.
     *  @param Il In-phase late correlator sum.
     *  @return Value near 1.0 when code phase is correctly aligned. */
    static double codeLockRatio(double Ie, double Ip, double Il);
};
}

#endif
