#include "GPSOpenClAcquisition.h"

using namespace GPSOpenCl;

Acquisition::Acquisition(Settings::Configuration conf)
{
    m_numberOfFreqencyBins =
        ((conf.acquisitionSettings.acquisitionDopplerMaximum - conf.acquisitionSettings.acquisitionDopplerMinimum) /
         conf.acquisitionSettings.acquisitionDopplerSearchRange) +
        1;
    m_initialFrequency = conf.acquisitionSettings.acquisitionDopplerMinimum;
    m_freqSpacing = conf.acquisitionSettings.acquisitionDopplerSearchRange;
    m_length = conf.rawDataSettings.numberOfSamplesPerCode;
    m_samplingFrequency = conf.rawDataSettings.samplingFrequency;

    createDopplerSearchTable();
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
        value = std::exp((IMAGINARY_UNIT * 2.0f * pi * frequency * sampleFloating * (1 / samplingRate)) + phaseOffset);
        output->push_back(value);
    }
}

void Acquisition::correlate(const ComplexFloatVector input, Compute *gpu, Code *code, Channel *acqChannel)
{
    float frequency = m_initialFrequency;
    float sumVal = 0.0f;
    float maxVal = 0.0f;
    int maxIndex = 0.0f;

    for (int freqBin = 0; freqBin < m_numberOfFreqencyBins; freqBin++)
    {
        ComplexFloatVector dopplerMultiplication;
        gpu->complexMultiplier(input, m_dopplerSearch[freqBin], &dopplerMultiplication);

        ComplexFloatVector dopplerMultiplicationFreq;
        gpu->fft(dopplerMultiplication, &dopplerMultiplicationFreq, Compute::FFTForward);

        ComplexFloatVector correlationFreq;
        gpu->complexMultiplier(code->m_upsampledFreqDomainCaCode[acqChannel->m_svId - 1], dopplerMultiplicationFreq,
                               &correlationFreq);

        ComplexFloatVector correlation;
        gpu->fft(correlationFreq, &correlation, Compute::FFTInverse);

        FloatVector correlationAbs;
        gpu->absolute(correlation, &correlationAbs);

        gpu->sum(correlationAbs, &sumVal);
        sumVal /= static_cast<float>(correlationAbs.size() * m_numberOfFreqencyBins);

        std::max_element(correlationAbs.begin(), correlationAbs.end());
        maxIndex = std::max_element(correlationAbs.begin(), correlationAbs.end()) - correlationAbs.begin();
        maxVal = correlationAbs.at(maxIndex);

        acqChannel->insertAcquisitionMetrics(maxVal, maxIndex, frequency, sumVal);

        frequency += m_freqSpacing;
    }
}
