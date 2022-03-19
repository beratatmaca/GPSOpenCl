#include "Acquisition.h"

#include <QElapsedTimer>

#include "../IOManagement/FileHandler.h"

GPSOpenCl::Acquisition::Acquisition(std::vector<std::complex<double>> code, 
                                    std::vector<std::complex<double>> rawData,
                                    double threshold,
                                    QObject *parent) : QThread(parent)
{
    m_code = code;
    m_rawData = rawData;

    m_snr = 0;
    m_peakFreq = 0;
    m_peakVal = 0;
    m_peakIndex = 0;
    m_snrThreshold = threshold;
    
    m_fftUtils = new FftUtils(4096);

    m_freqBins = 29;
    for (int i = 0; i < m_freqBins; i++)
    {
        freqList[i] = -7000.0 + i * 500.0;
    }
    
    m_logger = new Logger();
}

GPSOpenCl::Acquisition::~Acquisition()
{
    delete m_logger;
    delete m_fftUtils;
}

void GPSOpenCl::Acquisition::run()
{
    // Get high resolution timer
    QElapsedTimer timer;
    timer.start();
    std::vector<std::vector<double>> corrResult = correlator();
    checkResults(corrResult);
    qDebug() << "Acqusition operation took" << timer.nsecsElapsed() << "nanoseconds";
}

std::vector<std::vector<double>> GPSOpenCl::Acquisition::correlator()
{
    std::vector<std::vector<double>> result;

    std::vector<std::complex<double>> codeFFT = m_fftUtils->fftComplex(m_code);
    GPSOpenCl::Utils::conj(&codeFFT);

    for (int freqBin = 0; freqBin < m_freqBins; freqBin++)
    {
        std::vector<std::complex<double>> doppSignal = GPSOpenCl::Utils::exp(4096, freqList[freqBin], 4096000.0);
        std::vector<std::complex<double>> multArr;
        for (int j = 0; j < 4096; j++)
        {
            multArr.push_back(m_rawData.at(j) * doppSignal.at(j));
        }
        std::vector<std::complex<double>> multFFT = m_fftUtils->fftComplex(multArr);
        std::vector<std::complex<double>> convCode;
        for (int j = 0; j < 4096; j++)
        {
             convCode.push_back(codeFFT.at(j) * multFFT.at(j));
        }
        std::vector<std::complex<double>> convCodeIFFT = m_fftUtils->ifftComplex(convCode);
        std::vector<double> convCodeIFFTReal = GPSOpenCl::Utils::abs(convCodeIFFT);
        result.push_back(convCodeIFFTReal);
    }
    return result;
}

void GPSOpenCl::Acquisition::checkResults(std::vector<std::vector<double>> corrResult)
{
    double meanVal = 0;
    int maxFreqIndex = 0;

    for (int i = 0; i < m_freqBins; i++)
    {
        for(auto it = corrResult.at(i).begin(); it != corrResult.at(i).end(); ++it)
        {
            // Find max
            if (*it > m_peakVal)
            {
                m_peakVal = *it;
                m_peakIndex = std::distance(corrResult.at(i).begin(), it);
                maxFreqIndex = i;
            }
            // Find mean
            meanVal += *it;
        }
    }
    meanVal = meanVal/(corrResult.at(0).size()*m_freqBins);
    
    m_peakFreq = freqList[maxFreqIndex];
    m_snr = m_peakVal/meanVal;

    qDebug() << "Max value: " << m_peakVal << " at code index: " << m_peakIndex << "at freq" << m_peakFreq;
    qDebug() << "SNR : " << m_snr;

    if(m_snr > m_snrThreshold)
    {
        emit acquisitionSuccessful(m_peakFreq, m_peakIndex, m_snr);
    }
    else
    {
        emit acquisitionFailed();
    }
}
