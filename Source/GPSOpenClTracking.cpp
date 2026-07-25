#include "GPSOpenClTracking.h"

#include <algorithm>
#include <cmath>

using namespace GPSOpenCl;

Tracking::Tracking(Settings::Configuration conf)
    : m_gpu(nullptr),
      m_configuration(conf),
      m_totalSamples(0),
      m_pllTau1(0.0004494f),
      m_pllTau2(0.02998f),
      m_carrFreqBasis(0.0f),
      m_carrFreq(0.0f),
      m_remCarrPhase(0.0f),
      m_carrNco(0.0f),
      m_carrNcoPrev(0.0f),
      m_carrError(0.0f),
      m_carrErrorPrev(0.0f),
      m_dllTau1(0.07022f),
      m_dllTau2(0.37476f),
      m_codeFreqBasis(GPS_CA_CODE_FREQUENCY_HZ),
      m_codeFreq(GPS_CA_CODE_FREQUENCY_HZ),
      m_codePhaseStep(0.0f),
      m_remCodePhase(0.0f),
      m_codeNco(0.0f),
      m_codeNcoPrev(0.0f),
      m_codeError(0.0f),
      m_codeErrorPrev(0.0f),
      m_Ie(0.0f),
      m_Ip(0.0f),
      m_Il(0.0f),
      m_Qe(0.0f),
      m_Qp(0.0f),
      m_Ql(0.0f)
{
    m_gpu = new Compute();

    m_code.setConfiguration(m_configuration);

    m_totalSamples = m_configuration.rawDataSettings.numberOfSamplesPerCode;

    if (m_configuration.rawDataSettings.samplingFrequency > 0.0f)
    {
        m_codePhaseStep = m_codeFreq / m_configuration.rawDataSettings.samplingFrequency;
    }

    m_carrSig.resize(m_totalSamples);
    m_earlyCode.resize(m_totalSamples);
    m_promptCode.resize(m_totalSamples);
    m_lateCode.resize(m_totalSamples);
}

Tracking::~Tracking()
{
    if (m_gpu)
    {
        delete m_gpu;
        m_gpu = nullptr;
    }
}

void Tracking::initTrackingState(float initDopplerHz, float initCodePhaseChips)
{
    m_carrFreqBasis = initDopplerHz;
    m_carrFreq = initDopplerHz;
    m_remCarrPhase = 0.0f;
    m_carrNco = 0.0f;
    m_carrNcoPrev = 0.0f;
    m_carrError = 0.0f;
    m_carrErrorPrev = 0.0f;

    m_remCodePhase = initCodePhaseChips;
    m_codeFreqBasis = GPS_CA_CODE_FREQUENCY_HZ;
    m_codeFreq = GPS_CA_CODE_FREQUENCY_HZ;
    if (m_configuration.rawDataSettings.samplingFrequency > 0.0f)
    {
        m_codePhaseStep = m_codeFreq / m_configuration.rawDataSettings.samplingFrequency;
    }
    m_codeNco = 0.0f;
    m_codeNcoPrev = 0.0f;
    m_codeError = 0.0f;
    m_codeErrorPrev = 0.0f;

    resetAccumulation();
}

void Tracking::doWork(const ComplexFloatVector &input, int prn, ComplexFloatVector *output)
{
    earlyLatePromptGen(prn);
    numericOscillator();
    accumulator(input);
    freqDiscriminator();
    codeDiscriminator();

    if (output)
    {
        output->push_back(std::complex<float>(m_Ip, m_Qp));
    }

    resetAccumulation();
}

void Tracking::ncoMultiplicate(const ComplexFloatVector &input, float frequency, ComplexFloatVector *output)
{
    float samplingFreq = m_configuration.rawDataSettings.samplingFrequency;
    if (samplingFreq <= 0.0f) return;

    size_t length = input.size();
    FloatVector phaseVector(length, 0.0f);

    for (size_t i = 0; i < length; i++)
    {
        phaseVector[i] = static_cast<float>(2.0 * M_PI * frequency * static_cast<double>(i) / samplingFreq);
    }

    m_gpu->ncoMultiplication(input, phaseVector, output);
}

void Tracking::earlyLatePromptGen(int prn)
{
    int svIndex = std::clamp(prn - 1, 0, GPS_CA_SV_COUNT - 1);
    double phaseStep = m_remCodePhase;
    for (int i = 0; i < m_totalSamples; i++)
    {
        phaseStep = i * m_codePhaseStep + m_remCodePhase;

        int rawEarly = static_cast<int>(std::floor(phaseStep - 0.5 + 1023.0));
        int rawPrompt = static_cast<int>(std::floor(phaseStep));
        int rawLate = static_cast<int>(std::floor(phaseStep + 0.5));

        int earlyIndex = ((rawEarly % GPS_CA_CODE_LENGTH) + GPS_CA_CODE_LENGTH) % GPS_CA_CODE_LENGTH;
        int promptIndex = ((rawPrompt % GPS_CA_CODE_LENGTH) + GPS_CA_CODE_LENGTH) % GPS_CA_CODE_LENGTH;
        int lateIndex = ((rawLate % GPS_CA_CODE_LENGTH) + GPS_CA_CODE_LENGTH) % GPS_CA_CODE_LENGTH;

        m_earlyCode[i] = m_code.m_caCode[svIndex][earlyIndex];
        m_promptCode[i] = m_code.m_caCode[svIndex][promptIndex];
        m_lateCode[i] = m_code.m_caCode[svIndex][lateIndex];
    }
    m_remCodePhase = std::fmod(m_remCodePhase + m_totalSamples * m_codePhaseStep, 1023.0);
}

void Tracking::numericOscillator()
{
    float samplingFreq = m_configuration.rawDataSettings.samplingFrequency;
    if (samplingFreq <= 0.0f) return;

    for (int sample = 0; sample < m_totalSamples; sample++)
    {
        double sampDouble = static_cast<double>(sample);
        double phase = (2.0 * M_PI * m_carrFreq * sampDouble / samplingFreq) + m_remCarrPhase;
        m_carrSig[sample] = std::exp(IMAGINARY_UNIT * static_cast<float>(phase));
    }

    double finalPhase = (2.0 * M_PI * m_carrFreq * m_totalSamples / samplingFreq) + m_remCarrPhase;
    m_remCarrPhase = std::fmod(finalPhase, 2.0 * M_PI);
}

void Tracking::accumulator(const ComplexFloatVector &input)
{
    m_Ie = 0.0f; m_Qe = 0.0f;
    m_Ip = 0.0f; m_Qp = 0.0f;
    m_Il = 0.0f; m_Ql = 0.0f;

    size_t length = std::min(input.size(), static_cast<size_t>(m_totalSamples));

    for (size_t i = 0; i < length; i++)
    {
        std::complex<float> carrConj = std::conj(m_carrSig[i]);
        std::complex<float> wipeoff = input[i] * carrConj;

        float re = wipeoff.real();
        float im = wipeoff.imag();

        m_Ie += re * m_earlyCode[i];
        m_Qe += im * m_earlyCode[i];

        m_Ip += re * m_promptCode[i];
        m_Qp += im * m_promptCode[i];

        m_Il += re * m_lateCode[i];
        m_Ql += im * m_lateCode[i];
    }
}

void Tracking::freqDiscriminator()
{
    if (m_Ip != 0.0f)
    {
        m_carrError = std::atan2(m_Qp, m_Ip) / (2.0 * M_PI);
    }
    else
    {
        m_carrError = 0.0f;
    }

    if (m_pllTau1 != 0.0f)
    {
        m_carrNco = m_carrNcoPrev + (m_pllTau2 / m_pllTau1) * (m_carrError - m_carrErrorPrev) +
                    m_carrError * (GPS_CA_CODE_PERIOD_SEC / m_pllTau1);
    }

    m_carrNcoPrev = m_carrNco;
    m_carrErrorPrev = m_carrError;

    m_carrFreq = m_carrFreqBasis + m_carrNco;
}

void Tracking::codeDiscriminator()
{
    double earlyCoff = std::sqrt(m_Ie * m_Ie + m_Qe * m_Qe);
    double lateCoff = std::sqrt(m_Il * m_Il + m_Ql * m_Ql);
    double denom = earlyCoff + lateCoff;

    if (denom != 0.0)
    {
        m_codeError = (earlyCoff - lateCoff) / denom;
    }
    else
    {
        m_codeError = 0.0f;
    }

    if (m_dllTau1 != 0.0f)
    {
        m_codeNco = m_codeNcoPrev + (m_dllTau2 / m_dllTau1) * (m_codeError - m_codeErrorPrev) +
                    m_codeError * (GPS_CA_CODE_PERIOD_SEC / m_dllTau1);
    }

    m_codeNcoPrev = m_codeNco;
    m_codeErrorPrev = m_codeError;

    m_codeFreq = m_codeFreqBasis - m_codeNco;
    if (m_configuration.rawDataSettings.samplingFrequency > 0.0f)
    {
        m_codePhaseStep = m_codeFreq / m_configuration.rawDataSettings.samplingFrequency;
    }
}

void Tracking::resetAccumulation()
{
    m_Ie = 0.0f;
    m_Ip = 0.0f;
    m_Il = 0.0f;
    m_Qe = 0.0f;
    m_Qp = 0.0f;
    m_Ql = 0.0f;
}