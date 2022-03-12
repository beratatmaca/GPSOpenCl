#include "Acquisition.h"

#include "../IOManagement/FileHandler.h"
#include "../Utils/Utils.h"

GPSOpenCl::Acquisition::Acquisition(double code[], std::complex<double> rawData, QObject *parent) : QThread(parent)
{
    memcpy(&m_code[0], &code[0], sizeof(double) * 4096);
    memcpy(&m_rawData[0], &code[0], sizeof(std::complex<double>) * 4096);
    m_freqBins = 29;
    for (int i = 0 ; i<m_freqBins; i++)
    {
        freqList[i] = -7000.0 + i * 500.0;
    }
    m_logger = new Logger();
}

GPSOpenCl::Acquisition::~Acquisition()
{

}

void GPSOpenCl::Acquisition::run()
{
    std::complex<double>* codeFFT = GPSOpenCl::Utils::fftReal(m_code, 4096);
    codeFFT = GPSOpenCl::Utils::conj(codeFFT, 4096);

    for (int freqBin = 0; freqBin < m_freqBins; freqBin++)
    {
        std::complex<double>* doppSignal = GPSOpenCl::Utils::exp(4096, freqList[freqBin], 4096000.0);
        
        std::complex<double> multArr[4096];
        for (int j = 0 ; j<4096; j++)
        {
            multArr[j] = (m_rawData[j] * doppSignal[j]);
        }
        std::complex<double>* multFFT = GPSOpenCl::Utils::fftComplex(multArr, 4096);
        
        std::complex<double> convCode[4096];
        for (int j = 0 ; j<4096; j++)
        {
            convCode[j] = (codeFFT[j] * multFFT[j]);
        }

        std::complex<double>* convCodeIFFT = GPSOpenCl::Utils::ifftComplex(convCode, 4096);
        
        double* convCodeIFFTReal = GPSOpenCl::Utils::abs(convCodeIFFT, 4096);
        m_logger->log(convCodeIFFTReal, 4096);

        delete doppSignal;
        delete multFFT;
        delete convCodeIFFT;
    }
    delete codeFFT;
}
