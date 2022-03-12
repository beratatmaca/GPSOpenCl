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
        static std::complex<double>* fftReal(double input[], int length);
        static std::complex<double>* fftComplex(std::complex<double> input[], int length);
        static std::complex<double>* ifftComplex(std::complex<double> input[], int length);
        static std::complex<double>* conj(std::complex<double> input[], int length);
        static std::complex<double>* exp(int length, double frequency, double samplingRate);
        static double* abs(std::complex<double>* input, int length);
    };
}

#endif // ! INCLUDED_UTILS_H