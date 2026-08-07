#include "Acquisition/GPSOpenClAcquisition.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace GPSOpenCl;

Acquisition::Acquisition(const Settings::Configuration &conf) : Acquisition(conf.acquisitionInput)
{
}

Acquisition::Acquisition(const AcquisitionInput &input)
    : m_inputConfig(input),
      m_numberOfFreqencyBins(computeNumberOfFrequencyBins(input)),
      m_initialFrequencyHz(static_cast<float>(m_inputConfig.acquisitionDopplerMinimum)),
      m_freqSpacingHz(static_cast<float>(m_inputConfig.acquisitionDopplerSearchRange)),
      m_length(m_inputConfig.numberOfSamplesPerCode),
      m_samplingFrequencyHz(static_cast<float>(m_inputConfig.samplingFrequencyHz)),
      m_reuseFactor(computeReuseFactor())
{
    createDopplerSearchTable();

    m_binSlotShift.resize(static_cast<size_t>(m_numberOfFreqencyBins));
    for (int freqBin = 0; freqBin < m_numberOfFreqencyBins; freqBin++)
    {
        m_binSlotShift[static_cast<size_t>(freqBin)] = {freqBin % m_reuseFactor, freqBin / m_reuseFactor};
    }
}

Acquisition::~Acquisition() = default;

int Acquisition::computeNumberOfFrequencyBins(const AcquisitionInput &input)
{
    const int span = input.acquisitionDopplerMaximum - input.acquisitionDopplerMinimum;
    const int step = input.acquisitionDopplerSearchRange;

    if (span <= 0 || step <= 0)
    {
        return 1;
    }

    return ((span + step - 1) / step) + 1;
}

void Acquisition::createDopplerSearchTable()
{
    float frequency = m_initialFrequencyHz;

    for (int freqBin = 0; freqBin < m_reuseFactor; freqBin++)
    {
        m_dopplerSearch.emplace_back();

        exp(m_length, frequency, m_samplingFrequencyHz, 0.0, &m_dopplerSearch[freqBin]);

        frequency += m_freqSpacingHz;
    }
}

void Acquisition::exp(int length, float frequency, float samplingRateHz, float phaseOffset, ComplexFloatVector *output)
{
    const float pi = std::acos(-1.0f);
    std::complex<float> value;

    for (int sample = 0; sample < length; sample++)
    {
        auto sampleFloating = static_cast<float>(sample);
        value = std::exp(IMAGINARY_UNIT *
                         ((2.0f * pi * frequency * sampleFloating * (1.0f / samplingRateHz)) + phaseOffset));
        output->push_back(value);
    }
}

int Acquisition::computeReuseFactor() const
{
    const int perBinFallback = std::max(m_numberOfFreqencyBins, 1);

    if (m_length <= 0 || m_freqSpacingHz <= 0.0f || m_samplingFrequencyHz <= 0.0f)
    {
        return perBinFallback;
    }

    const float binResolution = m_samplingFrequencyHz / static_cast<float>(m_length);
    const float ratio = binResolution / m_freqSpacingHz;
    const int rounded = static_cast<int>(std::lround(ratio));

    if (rounded < 1 || rounded > m_numberOfFreqencyBins || std::fabs(ratio - static_cast<float>(rounded)) > 1e-3f)
    {
        return perBinFallback;
    }

    return rounded;
}

void Acquisition::correlate(const ComplexFloatVector &input,
                            SpectrumEngine *gpu,
                            CaCodeGenerator *code,
                            Channel *acqChannel)
{
    acqChannel->resetAcquisitionMetrics();

    gpu->invalidateResidentInput();

    for (int r = 0; r < m_reuseFactor; r++)
    {
        if (gpu->complexMultiplyThenFftToSlot(input, m_dopplerSearch[r], SpectrumEngine::FFTForward, r) != 0)
        {
            std::cerr << "Acquisition::correlate: forward FFT failed for SV " << acqChannel->svId
                      << " (samplesPerCode=" << m_length
                      << "; requires a power of two, otherwise the compute device reported an error); skipping "
                         "correlation"
                      << '\n';
            return;
        }
    }

    const ComplexFloatVector &codeSpectrum = code->upsampledFreqDomainCaCode[acqChannel->svId - 1];

    float bestPeakValue = 0.0f;
    int bestPeakIndex = 0;
    float bestPeakFrequency = m_initialFrequencyHz;
    float meanValueTotal = 0.0f;
    bool haveResult = false;

    auto accumulateBin = [&](const float *values, size_t count, float frequency)
    {
        float sumVal = 0.0f;
        float maxVal = values[0];
        int maxIndex = 0;
        for (size_t i = 0; i < count; i++)
        {
            const float value = values[i];
            sumVal += value;
            if (value > maxVal)
            {
                maxVal = value;
                maxIndex = static_cast<int>(i);
            }
        }
        meanValueTotal += sumVal / static_cast<float>(count * static_cast<size_t>(m_numberOfFreqencyBins));

        if (!haveResult || maxVal > bestPeakValue)
        {
            bestPeakValue = maxVal;
            bestPeakIndex = maxIndex;
            bestPeakFrequency = frequency;
            haveResult = true;
        }
    };

    m_batchAbs.clear();
    if (gpu->complexMultiplyResidentThenFftThenAbsoluteBatch(
            codeSpectrum, m_binSlotShift, SpectrumEngine::FFTInverse, &m_batchAbs) == 0 &&
        m_batchAbs.size() == static_cast<size_t>(m_numberOfFreqencyBins) * static_cast<size_t>(m_length))
    {
        float frequency = m_initialFrequencyHz;
        for (int freqBin = 0; freqBin < m_numberOfFreqencyBins; freqBin++)
        {
            const float *binValues =
                m_batchAbs.data() + (static_cast<size_t>(freqBin) * static_cast<size_t>(m_length));
            accumulateBin(binValues, static_cast<size_t>(m_length), frequency);
            frequency += m_freqSpacingHz;
        }
    }
    else
    {
        float frequency = m_initialFrequencyHz;
        FloatVector correlationAbs;
        for (int freqBin = 0; freqBin < m_numberOfFreqencyBins; freqBin++)
        {
            const int residue = freqBin % m_reuseFactor;
            const int shift = freqBin / m_reuseFactor;

            if (gpu->complexMultiplyResidentThenFftThenAbsolute(
                    codeSpectrum, residue, shift, SpectrumEngine::FFTInverse, &correlationAbs) != 0)
            {
                std::cerr << "Acquisition::correlate: inverse FFT failed (samplesPerCode=" << m_length
                          << " is not a power of two); skipping correlation for SV " << acqChannel->svId << '\n';
                return;
            }

            if (correlationAbs.empty())
            {
                std::cerr << "Acquisition::correlate: empty correlation result; skipping SV " << acqChannel->svId
                          << '\n';
                return;
            }

            accumulateBin(correlationAbs.data(), correlationAbs.size(), frequency);
            frequency += m_freqSpacingHz;
        }
    }

    if (haveResult)
    {
        acqChannel->insertAcquisitionMetrics(bestPeakValue, bestPeakIndex, bestPeakFrequency, meanValueTotal);
    }
}
