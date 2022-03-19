#include "Utils.h"

#include <clFFT.h> // for fft

GPSOpenCl::Utils::Utils()
{
}

GPSOpenCl::Utils::~Utils()
{
}

void GPSOpenCl::Utils::conj(std::vector<std::complex<double>> *input)
{
    for (auto it = input->begin(); it != input->end(); ++it)
    {
        // conjugate
        *it = std::conj(*it);
    }
}

std::vector<std::complex<double>> GPSOpenCl::Utils::exp(int length, double frequency, double samplingRate)
{
    std::vector<std::complex<double>> retVal(length);
    const std::complex<double> i(0, 1);
    const double pi = std::acos(-1);
    for (int sample = 0; sample < length; sample++)
    {
        double sampDouble = static_cast<double>(sample);
        retVal[sample] = std::exp(i * 2.0 * pi * frequency * sampDouble * (1 / samplingRate));
    }
    return retVal;
}

std::vector<double> GPSOpenCl::Utils::abs(std::vector<std::complex<double>> input)
{
    std::vector<double> retVal;
    for (auto it = input.begin(); it != input.end(); ++it)
    {
        retVal.push_back(std::abs(*it));
    }
    return retVal;
}
