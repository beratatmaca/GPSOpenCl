#include "GPSOpenClAcquisition.h"

#include <cmath>
#include <iostream>

using namespace GPSOpenCl;

Acquisition::Acquisition(Settings::Configuration conf)
    : m_inputConfig{STRUCT_VERSION_1,
                    conf.acquisitionSettings.acquisitionDopplerMinimum,
                    conf.acquisitionSettings.acquisitionDopplerMaximum,
                    conf.acquisitionSettings.acquisitionDopplerSearchRange,
                    conf.rawDataSettings.samplingFrequency,
                    conf.rawDataSettings.numberOfSamplesPerCode}
{
    m_numberOfFreqencyBins =
        ((m_inputConfig.acquisitionDopplerMaximum - m_inputConfig.acquisitionDopplerMinimum) /
         m_inputConfig.acquisitionDopplerSearchRange) +
        1;
    m_initialFrequency = static_cast<float>(m_inputConfig.acquisitionDopplerMinimum);
    m_freqSpacing = static_cast<float>(m_inputConfig.acquisitionDopplerSearchRange);
    m_length = m_inputConfig.numberOfSamplesPerCode;
    m_samplingFrequency = static_cast<float>(m_inputConfig.samplingFrequency);

    createDopplerSearchTable();
    m_reuseFactor = computeReuseFactor();
}

Acquisition::Acquisition(const AcquisitionInput &input)
    : m_inputConfig(input)
{
    m_numberOfFreqencyBins =
        ((m_inputConfig.acquisitionDopplerMaximum - m_inputConfig.acquisitionDopplerMinimum) /
         m_inputConfig.acquisitionDopplerSearchRange) +
        1;
    m_initialFrequency = static_cast<float>(m_inputConfig.acquisitionDopplerMinimum);
    m_freqSpacing = static_cast<float>(m_inputConfig.acquisitionDopplerSearchRange);
    m_length = m_inputConfig.numberOfSamplesPerCode;
    m_samplingFrequency = static_cast<float>(m_inputConfig.samplingFrequency);

    createDopplerSearchTable();
    m_reuseFactor = computeReuseFactor();
}

Acquisition::~Acquisition()
{
}

void Acquisition::createDopplerSearchTable()
{
    float frequency = m_initialFrequency;

    for (int freqBin = 0; freqBin < m_numberOfFreqencyBins; freqBin++)
    {
        m_dopplerSearch.push_back(ComplexFloatVector());

        exp(m_length, frequency, m_samplingFrequency, 0.0, &m_dopplerSearch[freqBin]);

        frequency += m_freqSpacing;
    }
}

void Acquisition::exp(int length, float frequency, float samplingRate, float phaseOffset, ComplexFloatVector *output)
{
    const float pi = std::acos(-1);
    std::complex<float> value;

    for (int sample = 0; sample < length; sample++)
    {
        float sampleFloating = static_cast<float>(sample);
        value = std::exp(IMAGINARY_UNIT * (2.0f * pi * frequency * sampleFloating * (1.0f / samplingRate) + phaseOffset));
        output->push_back(value);
    }
}

void Acquisition::correlate(const ComplexFloatVector &input, Compute *gpu, Code *code, Channel *acqChannel)
{
    acqChannel->resetAcquisitionMetrics();

    float frequency = m_initialFrequency;
    float maxVal = 0.0f;
    int maxIndex = 0;

    for (int freqBin = 0; freqBin < m_numberOfFreqencyBins; freqBin++)
    {
        float sumVal = 0.0f;

        ComplexFloatVector dopplerMultiplication;
        gpu->complexMultiplier(input, m_dopplerSearch[freqBin], &dopplerMultiplication);

        ComplexFloatVector dopplerMultiplicationFreq;
        if (gpu->fft(dopplerMultiplication, &dopplerMultiplicationFreq, Compute::FFTForward) != 0)
        {
            std::cerr << "Acquisition::correlate: forward FFT failed (samplesPerCode=" << m_length
                      << " is not a power of two); skipping correlation for SV " << acqChannel->m_svId << std::endl;
            return;
        }

        ComplexFloatVector correlationFreq;
        gpu->complexMultiplier(code->m_upsampledFreqDomainCaCode[acqChannel->m_svId - 1], dopplerMultiplicationFreq,
                               &correlationFreq);

        ComplexFloatVector correlation;
        if (gpu->fft(correlationFreq, &correlation, Compute::FFTInverse) != 0)
        {
            std::cerr << "Acquisition::correlate: inverse FFT failed (samplesPerCode=" << m_length
                      << " is not a power of two); skipping correlation for SV " << acqChannel->m_svId << std::endl;
            return;
        }

        FloatVector correlationAbs;
        gpu->absolute(correlation, &correlationAbs);

        if (correlationAbs.empty())
        {
            std::cerr << "Acquisition::correlate: empty correlation result; skipping SV " << acqChannel->m_svId
                      << std::endl;
            return;
        }

        gpu->sum(correlationAbs, &sumVal);
        sumVal /= static_cast<float>(correlationAbs.size() * m_numberOfFreqencyBins);

        auto maxIt = std::max_element(correlationAbs.begin(), correlationAbs.end());
        maxIndex = maxIt - correlationAbs.begin();
        maxVal = correlationAbs.at(maxIndex);

        acqChannel->insertAcquisitionMetrics(maxVal, maxIndex, frequency, sumVal);

        frequency += m_freqSpacing;
    }
}
