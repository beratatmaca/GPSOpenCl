#include "Tracking/GPSOpenClLockDetector.hpp"

#include <cmath>

using namespace GPSOpenCl;

double LockDetector::carrierLockIndicator(double Ip, double Qp)
{
    const double denom = (Ip * Ip) + (Qp * Qp);
    if (denom < 1e-12)
    {
        return 0.0;
    }
    return (Ip * Ip - Qp * Qp) / denom;
}

double LockDetector::codeLockRatio(double Ie, double Ip, double Il)
{
    const double absIp = std::fabs(Ip);
    if (absIp < 1e-12)
    {
        return 0.0;
    }
    return (std::fabs(Ie) + std::fabs(Il)) / absIp;
}
