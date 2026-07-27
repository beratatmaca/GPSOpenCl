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

int Acquisition::computeReuseFactor() const
{
    if (m_length <= 0 || m_freqSpacing <= 0.0f || m_samplingFrequency <= 0.0f)
    {
        return 1;
    }

    float binResolution = m_samplingFrequency / static_cast<float>(m_length);
    float ratio = binResolution / m_freqSpacing;
    int rounded = static_cast<int>(std::lround(ratio));

    if (rounded < 1 || rounded > m_numberOfFreqencyBins || std::fabs(ratio - static_cast<float>(rounded)) > 1e-3f)
    {
        return 1;
    }

    return rounded;
}

void Acquisition::circularShiftFreqDomain(const ComplexFloatVector &input, int shiftBins, ComplexFloatVector *output)
{
    int n = static_cast<int>(input.size());
    output->resize(static_cast<size_t>(n));
    if (n == 0)
    {
        return;
    }

    int shift = ((shiftBins % n) + n) % n;
    auto splitPoint = input.end() - shift;
    std::copy(input.begin(), splitPoint, output->begin() + shift);
    std::copy(splitPoint, input.end(), output->begin());
}

void Acquisition::correlate(const ComplexFloatVector &input, Compute *gpu, Code *code, Channel *acqChannel)
{
    acqChannel->resetAcquisitionMetrics();

    // Reference spectra: one forward FFT per residue class (see computeReuseFactor). Every other bin's
    // carrier-wiped spectrum is derived from these via an exact circular shift instead of its own FFT.
    std::vector<ComplexFloatVector> referenceFreq(static_cast<size_t>(m_reuseFactor));
    for (int r = 0; r < m_reuseFactor; r++)
    {
        if (gpu->complexMultiplyThenFft(input, m_dopplerSearch[r], Compute::FFTForward, &referenceFreq[r]) != 0)
        {
            std::cerr << "Acquisition::correlate: forward FFT failed (samplesPerCode=" << m_length
                      << " is not a power of two); skipping correlation for SV " << acqChannel->m_svId << std::endl;
            return;
        }
    }

    float frequency = m_initialFrequency;

    for (int freqBin = 0; freqBin < m_numberOfFreqencyBins; freqBin++)
    {
        float sumVal = 0.0f;
        int residue = freqBin % m_reuseFactor;
        int shift = freqBin / m_reuseFactor;

        const ComplexFloatVector *dopplerMultiplicationFreq = &referenceFreq[residue];
        ComplexFloatVector shiftedFreq;
        if (shift > 0)
        {
            circularShiftFreqDomain(referenceFreq[residue], shift, &shiftedFreq);
            dopplerMultiplicationFreq = &shiftedFreq;
        }

        FloatVector correlationAbs;
        if (gpu->complexMultiplyThenFftThenAbsolute(code->m_upsampledFreqDomainCaCode[acqChannel->m_svId - 1],
                                                    *dopplerMultiplicationFreq, Compute::FFTInverse,
                                                    &correlationAbs) != 0)
        {
            std::cerr << "Acquisition::correlate: inverse FFT failed (samplesPerCode=" << m_length
                      << " is not a power of two); skipping correlation for SV " << acqChannel->m_svId << std::endl;
            return;
        }

        if (correlationAbs.empty())
        {
            std::cerr << "Acquisition::correlate: empty correlation result; skipping SV " << acqChannel->m_svId
                      << std::endl;
            return;
        }

        gpu->sum(correlationAbs, &sumVal);
        sumVal /= static_cast<float>(correlationAbs.size() * m_numberOfFreqencyBins);

        auto maxIt = std::max_element(correlationAbs.begin(), correlationAbs.end());
        int maxIndex = static_cast<int>(maxIt - correlationAbs.begin());
        float maxVal = correlationAbs.at(static_cast<size_t>(maxIndex));

        acqChannel->insertAcquisitionMetrics(maxVal, maxIndex, frequency, sumVal);

        frequency += m_freqSpacing;
    }
}
