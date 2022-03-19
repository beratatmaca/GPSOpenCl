#ifndef INCLUDED_UTILS_H
#define INCLUDED_UTILS_H

#include <complex>
#include <QDebug>

namespace GPSOpenCl
{
    class Utils
    {
    public:
        Utils();
        ~Utils();
        static void conj(std::vector<std::complex<double>> *input);
        static std::vector<std::complex<double>> exp(int length, double frequency, double samplingRate);
        static std::vector<double> abs(std::vector<std::complex<double>> input);
    };
}

#endif // ! INCLUDED_UTILS_H