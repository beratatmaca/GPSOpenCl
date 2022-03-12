#include "Utils.h"

GPSOpenCl::Utils::Utils()
{

}

GPSOpenCl::Utils::~Utils()
{

}

std::complex<double>* GPSOpenCl::Utils::fftReal(double input[], int length)
{
    std::complex<double>* output = new std::complex<double>[length];
    for (int i = 0; i < length; i++)
    {
        output[i] = std::complex<double>(input[i], 0);
    }
    return output;
}

std::complex<double>* GPSOpenCl::Utils::fftComplex(std::complex<double> input[], int length)
{
    std::complex<double>* output = new std::complex<double>[length];
    for (int i = 0; i < length; i++)
    {
        output[i] = input[i];
    }
    return output;
}

std::complex<double>* GPSOpenCl::Utils::ifftComplex(std::complex<double> input[], int length)
{
    std::complex<double>* output = new std::complex<double>[length];
    for (int i = 0; i < length; i++)
    {
        output[i] = input[i];
    }
    return output;
}

std::complex<double>* GPSOpenCl::Utils::conj(std::complex<double> input[], int length)
{
    std::complex<double>* output = new std::complex<double>[length];
    for (int i = 0; i < length; i++)
    {
        output[i] = input[i];
    }
    return output;
}
