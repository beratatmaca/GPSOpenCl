#include "GPSOpenClAcquisition.h"

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace GPSOpenCl;

Acquisition::Acquisition(const Settings::Configuration &conf)
    : m_inputConfig(conf.acquisitionInput),
      m_numberOfFreqencyBins(((m_inputConfig.acquisitionDopplerMaximum - m_inputConfig.acquisitionDopplerMinimum) /
                              m_inputConfig.acquisitionDopplerSearchRange) +
                             1),
      m_initialFrequency(static_cast<float>(m_inputConfig.acquisitionDopplerMinimum)),
      m_freqSpacing(static_cast<float>(m_inputConfig.acquisitionDopplerSearchRange)),
      m_length(m_inputConfig.numberOfSamplesPerCode),
      m_samplingFrequency(static_cast<float>(m_inputConfig.samplingFrequency)),
      m_reuseFactor(computeReuseFactor())
{

    createDopplerSearchTable();
}

Acquisition::Acquisition(const AcquisitionInput &input)
    : m_inputConfig(input),
      m_numberOfFreqencyBins(((m_inputConfig.acquisitionDopplerMaximum - m_inputConfig.acquisitionDopplerMinimum) /
                              m_inputConfig.acquisitionDopplerSearchRange) +
                             1),
      m_initialFrequency(static_cast<float>(m_inputConfig.acquisitionDopplerMinimum)),
      m_freqSpacing(static_cast<float>(m_inputConfig.acquisitionDopplerSearchRange)),
      m_length(m_inputConfig.numberOfSamplesPerCode),
      m_samplingFrequency(static_cast<float>(m_inputConfig.samplingFrequency)),
      m_reuseFactor(computeReuseFactor())
{

    createDopplerSearchTable();
}

Acquisition::~Acquisition() = default;

void Acquisition::createDopplerSearchTable()
{
    float frequency = m_initialFrequency;

    for (int freqBin = 0; freqBin < m_numberOfFreqencyBins; freqBin++)
    {
        m_dopplerSearch.emplace_back();

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
        auto sampleFloating = static_cast<float>(sample);
        value =
            std::exp(IMAGINARY_UNIT * ((2.0f * pi * frequency * sampleFloating * (1.0f / samplingRate)) + phaseOffset));
        output->push_back(value);
    }
}

int Acquisition::computeReuseFactor() const
{
    const int perBinFallback = std::max(m_numberOfFreqencyBins, 1);

    if (m_length <= 0 || m_freqSpacing <= 0.0f || m_samplingFrequency <= 0.0f)
    {
        return perBinFallback;
    }

    const float binResolution = m_samplingFrequency / static_cast<float>(m_length);
    const float ratio = binResolution / m_freqSpacing;
    const int rounded = static_cast<int>(std::lround(ratio));

    if (rounded < 1 || rounded > m_numberOfFreqencyBins || std::fabs(ratio - static_cast<float>(rounded)) > 1e-3f)
    {
        return perBinFallback;
    }

    return rounded;
}

void Acquisition::correlate(const ComplexFloatVector &input, Compute *gpu, Code *code, Channel *acqChannel)
{
    acqChannel->resetAcquisitionMetrics();

    // Reference spectra: one forward FFT per residue class (see computeReuseFactor), left resident
    // in a compute slot (device buffer on GPU). Every other bin's carrier-wiped spectrum is derived
    // from its residue's slot via an exact circular shift applied as an index offset during the
    // multiply, so per-bin work involves no host-side shift, copy, or re-upload.
    for (int r = 0; r < m_reuseFactor; r++)
    {
        if (gpu->complexMultiplyThenFftToSlot(input, m_dopplerSearch[r], Compute::FFTForward, r) != 0)
        {
            std::cerr << "Acquisition::correlate: forward FFT failed for SV " << acqChannel->m_svId
                      << " (samplesPerCode=" << m_length
                      << "; requires a power of two, otherwise the compute device reported an error); skipping "
                         "correlation"
                      << '\n';
            return;
        }
    }

    const ComplexFloatVector &codeSpectrum = code->m_upsampledFreqDomainCaCode[acqChannel->m_svId - 1];

    float bestPeakValue = 0.0f;
    int bestPeakIndex = 0;
    float bestPeakFrequency = m_initialFrequency;
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

    // One GPU submission covering every Doppler bin with a single readback; falls back to one
    // round-trip per bin when the device or slot state cannot support the batch.
    std::vector<std::pair<int, int>> binSlotShift(static_cast<size_t>(m_numberOfFreqencyBins));
    for (int freqBin = 0; freqBin < m_numberOfFreqencyBins; freqBin++)
    {
        binSlotShift[static_cast<size_t>(freqBin)] = {freqBin % m_reuseFactor, freqBin / m_reuseFactor};
    }

    FloatVector batchAbs;
    if (gpu->complexMultiplyResidentThenFftThenAbsoluteBatch(
            codeSpectrum, binSlotShift, Compute::FFTInverse, &batchAbs) == 0 &&
        batchAbs.size() == static_cast<size_t>(m_numberOfFreqencyBins) * static_cast<size_t>(m_length))
    {
        float frequency = m_initialFrequency;
        for (int freqBin = 0; freqBin < m_numberOfFreqencyBins; freqBin++)
        {
            const float *binValues = batchAbs.data() + (static_cast<size_t>(freqBin) * static_cast<size_t>(m_length));
            accumulateBin(binValues, static_cast<size_t>(m_length), frequency);
            frequency += m_freqSpacing;
        }
    }
    else
    {
        float frequency = m_initialFrequency;
        FloatVector correlationAbs;
        for (int freqBin = 0; freqBin < m_numberOfFreqencyBins; freqBin++)
        {
            const int residue = freqBin % m_reuseFactor;
            const int shift = freqBin / m_reuseFactor;

            if (gpu->complexMultiplyResidentThenFftThenAbsolute(
                    codeSpectrum, residue, shift, Compute::FFTInverse, &correlationAbs) != 0)
            {
                std::cerr << "Acquisition::correlate: inverse FFT failed (samplesPerCode=" << m_length
                          << " is not a power of two); skipping correlation for SV " << acqChannel->m_svId << '\n';
                return;
            }

            if (correlationAbs.empty())
            {
                std::cerr << "Acquisition::correlate: empty correlation result; skipping SV " << acqChannel->m_svId
                          << '\n';
                return;
            }

            accumulateBin(correlationAbs.data(), correlationAbs.size(), frequency);
            frequency += m_freqSpacing;
        }
    }

    if (haveResult)
    {
        acqChannel->insertAcquisitionMetrics(bestPeakValue, bestPeakIndex, bestPeakFrequency, meanValueTotal);
    }
}
