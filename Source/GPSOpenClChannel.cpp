#include "GPSOpenClChannel.h"
#include "GPSOpenClCommon.h"

#include <iostream>

using namespace GPSOpenCl;

Channel::Channel()
{
    m_svId = 0;

    m_acquisitionPeakIndex = 0;
    m_acquisitionPeakValue = 0.0f;
    m_acquisitionPeakFrequency = 0.0f;
    m_acquisitionMeanValue = 0.0f;
    m_acquisitionCN0 = 0.0f;
    m_acquisitionPeakRatio = 0.0f;
    m_acquisitionProcessingGain = 10.0 * std::log10(GPS_CA_CODE_FREQUENCY_HZ / GPS_CA_CODE_LENGTH);
}

Channel::~Channel()
{
}

void Channel::insertAcquisitionMetrics(float peakValue, int peakIndex, float peakFrequency, float meanValue)
{
    if (peakValue > m_acquisitionPeakValue)
    {
        m_acquisitionPeakValue = peakValue;
        m_acquisitionPeakIndex = peakIndex;
        m_acquisitionPeakFrequency = peakFrequency;
    }

    m_acquisitionMeanValue += meanValue;
    if (meanValue != 0.0f)
    {
        m_acquisitionPeakRatio = m_acquisitionPeakValue / m_acquisitionMeanValue;
        m_acquisitionCN0 = 10 * std::log10(m_acquisitionPeakRatio) + m_acquisitionProcessingGain;
    }
    else
    {
        m_acquisitionPeakRatio = 0.0f;
        m_acquisitionCN0 = 0.0f;
    }
}

void Channel::checkAcquisition()
{
    std::cout << "SV ID " << m_svId << " C/N0 : " << m_acquisitionCN0 << std::endl;
}

void Channel::getAcquisitionResults(int *peakIndex, float *peakValue, float *peakFrequency, float *meanValue,
                                    float *cno, float *peakRatio)
{
    *peakIndex = m_acquisitionPeakIndex;
    *peakValue = m_acquisitionPeakValue;
    *peakFrequency = m_acquisitionPeakFrequency;
    *meanValue = m_acquisitionMeanValue;
    *cno = m_acquisitionCN0;
    *peakRatio = m_acquisitionPeakRatio;
}