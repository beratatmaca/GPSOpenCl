#include "GPSOpenClTracking.h"

#include "GPSOpenClLockDetector.h"

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace GPSOpenCl;

float Tracking::loopFilterTau1(double noiseBandwidthHz)
{
    const double zeta = 0.70710678118654752440;
    double wn = noiseBandwidthHz * 8.0 * zeta / (4.0 * zeta * zeta + 1.0);
    return static_cast<float>(1.0 / (wn * wn));
}

float Tracking::loopFilterTau2(double noiseBandwidthHz)
{
    const double zeta = 0.70710678118654752440;
    double wn = noiseBandwidthHz * 8.0 * zeta / (4.0 * zeta * zeta + 1.0);
    return static_cast<float>(2.0 * zeta / wn);
}

Tracking::Tracking(Settings::Configuration conf)
    : m_configuration(conf),
      m_inputConfig{STRUCT_VERSION_1, conf.trackingInput.pllBandwidthHz, conf.trackingInput.dllBandwidthHz, conf.rawDataSettings.samplingFrequency, conf.rawDataSettings.numberOfSamplesPerCode},
      m_totalSamples(0),
      m_pllTau1(loopFilterTau1(conf.trackingInput.pllBandwidthHz)),
      m_pllTau2(loopFilterTau2(conf.trackingInput.pllBandwidthHz)),
      m_carrFreqBasis(0.0f),
      m_carrFreq(0.0f),
      m_remCarrPhase(0.0f),
      m_carrNco(0.0f),
      m_carrNcoPrev(0.0f),
      m_carrError(0.0f),
      m_carrErrorPrev(0.0f),
      m_fllGain(4.0f * static_cast<float>(conf.trackingInput.fllBandwidthHz) * static_cast<float>(GPS_CA_CODE_PERIOD_SEC)),
      m_rateAidGain(4.0f * static_cast<float>(conf.trackingInput.rateAidBandwidthHz) * static_cast<float>(GPS_CA_CODE_PERIOD_SEC)),
      m_fllNco(0.0f),
      m_ipPrev(0.0f),
      m_qpPrev(0.0f),
      m_promptMagnitudeEma(0.0f),
      m_blocksSinceInit(0),
      m_fllPullInBlocks(conf.trackingInput.fllPullInBlocks),
      m_dllTau1(loopFilterTau1(conf.trackingInput.dllBandwidthHz)),
      m_dllTau2(loopFilterTau2(conf.trackingInput.dllBandwidthHz)),
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
      m_Ql(0.0f),
      m_carrierLockEma(0.0f),
      m_codeLockEma(0.0f),
      m_lastChannelState(0)
{
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

Tracking::Tracking(const TrackingInput &input)
    : m_inputConfig(input),
      m_totalSamples(0),
      m_pllTau1(loopFilterTau1(input.pllBandwidthHz)),
      m_pllTau2(loopFilterTau2(input.pllBandwidthHz)),
      m_carrFreqBasis(0.0f),
      m_carrFreq(0.0f),
      m_remCarrPhase(0.0f),
      m_carrNco(0.0f),
      m_carrNcoPrev(0.0f),
      m_carrError(0.0f),
      m_carrErrorPrev(0.0f),
      m_fllGain(4.0f * static_cast<float>(input.fllBandwidthHz) * static_cast<float>(GPS_CA_CODE_PERIOD_SEC)),
      m_rateAidGain(4.0f * static_cast<float>(input.rateAidBandwidthHz) * static_cast<float>(GPS_CA_CODE_PERIOD_SEC)),
      m_fllNco(0.0f),
      m_ipPrev(0.0f),
      m_qpPrev(0.0f),
      m_promptMagnitudeEma(0.0f),
      m_blocksSinceInit(0),
      m_fllPullInBlocks(input.fllPullInBlocks),
      m_dllTau1(loopFilterTau1(input.dllBandwidthHz)),
      m_dllTau2(loopFilterTau2(input.dllBandwidthHz)),
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
      m_Ql(0.0f),
      m_carrierLockEma(0.0f),
      m_codeLockEma(0.0f),
      m_lastChannelState(0)
{
    m_configuration.trackingInput = input;
    m_configuration.rawDataSettings.samplingFrequency = static_cast<float>(input.samplingFrequency);
    m_configuration.rawDataSettings.numberOfSamplesPerCode = input.numberOfSamplesPerCode;

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

    m_fllNco = 0.0f;
    m_ipPrev = 0.0f;
    m_qpPrev = 0.0f;
    m_promptMagnitudeEma = 0.0f;
    m_blocksSinceInit = 0;

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

    m_carrierLockEma = 0.0f;
    m_codeLockEma = 0.0f;

    resetAccumulation();
}

void Tracking::doWork(const ComplexFloatVector &input, int prn, ComplexFloatVector *output, uint32_t channelState)
{
    m_lastChannelState = channelState;

    auto subStageT0 = std::chrono::high_resolution_clock::now();
    earlyLatePromptGen(prn);
    auto subStageT1 = std::chrono::high_resolution_clock::now();
    numericOscillator();
    auto subStageT2 = std::chrono::high_resolution_clock::now();
    accumulator(input);
    auto subStageT3 = std::chrono::high_resolution_clock::now();

    m_earlyLatePromptGenTimeMs = std::chrono::duration<float, std::milli>(subStageT1 - subStageT0).count();
    m_numericOscillatorTimeMs = std::chrono::duration<float, std::milli>(subStageT2 - subStageT1).count();
    m_accumulatorTimeMs = std::chrono::duration<float, std::milli>(subStageT3 - subStageT2).count();

    if (m_blocksSinceInit < m_fllPullInBlocks)
    {
        if (isPromptSignalReliable())
        {
            fllDiscriminator();
        }
        else
        {
            m_carrFreq = m_carrFreqBasis + m_fllNco;
        }
    }
    else
    {
        if (m_blocksSinceInit == m_fllPullInBlocks)
        {
            m_carrNco = m_fllNco;
            m_carrNcoPrev = m_fllNco;
            m_carrErrorPrev = computeCostasPhaseError();
        }

        if (isPromptSignalReliable())
        {
            rateAidDiscriminator();
            freqDiscriminator();
        }
        else
        {
            m_carrFreq = m_carrFreqBasis + m_carrNco;
        }
    }
    codeDiscriminator();
    updateLockIndicators();
    m_blocksSinceInit++;

    if (output)
    {
        output->push_back(std::complex<float>(m_Ip, m_Qp));
    }

    if (m_sink)
    {
        TrackingOutput trkOut = getTrackingOutput(prn);
        m_sink->publishTrackingOutput(trkOut);
    }

    resetAccumulation();
}

TrackingOutput Tracking::getTrackingOutput(int prn) const
{
    TrackingOutput out{};
    out.structVersion = STRUCT_VERSION_1;
    out.prn = prn;
    out.carrierFreqHz = static_cast<double>(m_carrFreq);
    out.codeFreqHz = static_cast<double>(m_codeFreq);
    out.carrierError = static_cast<double>(m_carrError);
    out.codeError = static_cast<double>(m_codeError);
    out.Ie = static_cast<double>(m_Ie);
    out.Ip = static_cast<double>(m_Ip);
    out.Il = static_cast<double>(m_Il);
    out.Qe = static_cast<double>(m_Qe);
    out.Qp = static_cast<double>(m_Qp);
    out.Ql = static_cast<double>(m_Ql);
    out.channelState = m_lastChannelState;
    out.carrierLockIndicator = static_cast<double>(m_carrierLockEma);
    out.codeLockRatio = static_cast<double>(m_codeLockEma);
    return out;
}

void Tracking::getSubStageTimings(float *earlyLatePromptGenMs, float *numericOscillatorMs, float *accumulatorMs) const
{
    *earlyLatePromptGenMs = m_earlyLatePromptGenTimeMs;
    *numericOscillatorMs = m_numericOscillatorTimeMs;
    *accumulatorMs = m_accumulatorTimeMs;
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

    double phaseStepRad = 2.0 * M_PI * static_cast<double>(m_carrFreq) / samplingFreq;

    std::complex<float> step = std::exp(IMAGINARY_UNIT * static_cast<float>(phaseStepRad));
    std::complex<float> phasor = std::exp(IMAGINARY_UNIT * m_remCarrPhase);

    for (int sample = 0; sample < m_totalSamples; sample++)
    {
        m_carrSig[sample] = phasor;
        phasor *= step;
    }

    double finalPhase = (phaseStepRad * m_totalSamples) + m_remCarrPhase;
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

float Tracking::computeFllError() const
{
    double cross = static_cast<double>(m_ipPrev) * m_Qp - static_cast<double>(m_Ip) * m_qpPrev;
    double dot = static_cast<double>(m_ipPrev) * m_Ip + static_cast<double>(m_qpPrev) * m_Qp;

    if (cross == 0.0 && dot == 0.0)
    {
        return 0.0f;
    }

    return static_cast<float>(std::atan2(cross, dot) / (2.0 * M_PI * GPS_CA_CODE_PERIOD_SEC));
}

void Tracking::fllDiscriminator()
{
    float fllError = (m_blocksSinceInit > 0) ? computeFllError() : 0.0f;
    m_fllNco += fllError * m_fllGain;

    m_ipPrev = m_Ip;
    m_qpPrev = m_Qp;

    m_carrFreq = m_carrFreqBasis + m_fllNco;
}

bool Tracking::isPromptSignalReliable()
{
    double currentMagnitude = std::sqrt(static_cast<double>(m_Ip) * m_Ip + static_cast<double>(m_Qp) * m_Qp);
    bool reliable = (m_promptMagnitudeEma <= 0.0f) || (currentMagnitude >= 0.3 * m_promptMagnitudeEma);

    if (reliable)
    {
        m_promptMagnitudeEma = static_cast<float>(0.05 * currentMagnitude + 0.95 * m_promptMagnitudeEma);
    }

    return reliable;
}

void Tracking::rateAidDiscriminator()
{
    float bleed = m_rateAidGain * m_carrNco;
    m_carrFreqBasis += bleed;
    m_carrFreqBasis = std::clamp(m_carrFreqBasis, -15000.0f, 15000.0f);
    m_carrNco -= bleed;
    m_carrNcoPrev = m_carrNco;
}

float Tracking::computeCostasPhaseError() const
{
    if (m_Ip == 0.0f && m_Qp == 0.0f)
    {
        return 0.0f;
    }

    return static_cast<float>(std::atan2(2.0 * m_Qp * m_Ip, m_Ip * m_Ip - m_Qp * m_Qp) / (4.0 * M_PI));
}

void Tracking::freqDiscriminator()
{
    m_carrError = computeCostasPhaseError();

    if (m_pllTau1 != 0.0f)
    {
        m_carrNco = m_carrNcoPrev + (m_pllTau2 / m_pllTau1) * (m_carrError - m_carrErrorPrev) +
                    m_carrError * (GPS_CA_CODE_PERIOD_SEC / m_pllTau1);
    }

    m_carrNcoPrev = m_carrNco;
    m_carrErrorPrev = m_carrError;

    m_ipPrev = m_Ip;
    m_qpPrev = m_Qp;

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

void Tracking::updateLockIndicators()
{
    double alpha = m_inputConfig.lockIndicatorEmaAlpha;

    double instantCarrierLock = LockDetector::carrierLockIndicator(m_Ip, m_Qp);
    double instantCodeLock = LockDetector::codeLockRatio(m_Ie, m_Ip, m_Il);

    m_carrierLockEma = static_cast<float>(alpha * instantCarrierLock + (1.0 - alpha) * m_carrierLockEma);
    m_codeLockEma = static_cast<float>(alpha * instantCodeLock + (1.0 - alpha) * m_codeLockEma);
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
