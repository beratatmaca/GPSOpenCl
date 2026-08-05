#include "GPSOpenClTracking.h"

#include "GPSOpenClLockDetector.h"

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace GPSOpenCl;

float Tracking::loopFilterTau1(double noiseBandwidthHz)
{
    const double zeta = 0.70710678118654752440;
    const double wn = noiseBandwidthHz * 8.0 * zeta / (4.0 * zeta * zeta + 1.0);
    return static_cast<float>(1.0 / (wn * wn));
}

float Tracking::loopFilterTau2(double noiseBandwidthHz)
{
    const double zeta = 0.70710678118654752440;
    const double wn = noiseBandwidthHz * 8.0 * zeta / (4.0 * zeta * zeta + 1.0);
    return static_cast<float>(2.0 * zeta / wn);
}

Tracking::Tracking(const Settings::Configuration &conf)
    : m_configuration(conf),
      m_inputConfig{STRUCT_VERSION_1,
                    conf.trackingInput.pllBandwidthHz,
                    conf.trackingInput.dllBandwidthHz,
                    conf.acquisitionInput.samplingFrequencyHz,
                    conf.acquisitionInput.numberOfSamplesPerCode},
      m_totalSamples(conf.acquisitionInput.numberOfSamplesPerCode),
      m_pllTau1(loopFilterTau1(conf.trackingInput.pllBandwidthHz)),
      m_pllTau2(loopFilterTau2(conf.trackingInput.pllBandwidthHz)),
      m_carrFreqBasisHz(0.0f),
      m_carrFreqHz(0.0f),
      m_remCarrPhase(0.0f),
      m_carrNco(0.0f),
      m_carrNcoPrev(0.0f),
      m_carrErrorCycles(0.0f),
      m_carrErrorPrevCycles(0.0f),
      m_fllGain(4.0f * static_cast<float>(conf.trackingInput.fllBandwidthHz) *
                static_cast<float>(GPS_CA_CODE_PERIOD_SEC)),
      m_rateAidGain(4.0f * static_cast<float>(conf.trackingInput.rateAidBandwidthHz) *
                    static_cast<float>(GPS_CA_CODE_PERIOD_SEC)),
      m_fllNco(0.0f),
      m_ipPrev(0.0f),
      m_qpPrev(0.0f),
      m_promptMagnitudeEma(0.0f),
      m_blocksSinceInit(0),
      m_fllPullInBlocks(conf.trackingInput.fllPullInBlocks),
      m_dllTau1(loopFilterTau1(conf.trackingInput.dllBandwidthHz)),
      m_dllTau2(loopFilterTau2(conf.trackingInput.dllBandwidthHz)),
      m_codeFreqBasisHz(GPS_CA_CODE_FREQUENCY_HZ),
      m_codeFreqHz(GPS_CA_CODE_FREQUENCY_HZ),
      m_codePhaseStep(0.0f),
      m_remCodePhase(0.0f),
      m_codeNco(0.0f),
      m_codeNcoPrev(0.0f),
      m_codeErrorChips(0.0f),
      m_codeErrorPrevChips(0.0f),
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

    if (m_configuration.acquisitionInput.samplingFrequencyHz > 0.0f)
    {
        m_codePhaseStep = m_codeFreqHz / m_configuration.acquisitionInput.samplingFrequencyHz;
    }
}

Tracking::Tracking(const TrackingInput &input)
    : m_inputConfig(input),
      m_totalSamples(input.numberOfSamplesPerCode),
      m_pllTau1(loopFilterTau1(input.pllBandwidthHz)),
      m_pllTau2(loopFilterTau2(input.pllBandwidthHz)),
      m_carrFreqBasisHz(0.0f),
      m_carrFreqHz(0.0f),
      m_remCarrPhase(0.0f),
      m_carrNco(0.0f),
      m_carrNcoPrev(0.0f),
      m_carrErrorCycles(0.0f),
      m_carrErrorPrevCycles(0.0f),
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
      m_codeFreqBasisHz(GPS_CA_CODE_FREQUENCY_HZ),
      m_codeFreqHz(GPS_CA_CODE_FREQUENCY_HZ),
      m_codePhaseStep(0.0f),
      m_remCodePhase(0.0f),
      m_codeNco(0.0f),
      m_codeNcoPrev(0.0f),
      m_codeErrorChips(0.0f),
      m_codeErrorPrevChips(0.0f),
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
    m_configuration.acquisitionInput.samplingFrequencyHz = static_cast<float>(input.samplingFrequencyHz);
    m_configuration.acquisitionInput.numberOfSamplesPerCode = input.numberOfSamplesPerCode;

    m_code.setConfiguration(m_configuration);

    if (m_configuration.acquisitionInput.samplingFrequencyHz > 0.0f)
    {
        m_codePhaseStep = m_codeFreqHz / m_configuration.acquisitionInput.samplingFrequencyHz;
    }
}

Tracking::~Tracking() = default;

void Tracking::initTrackingState(float initDopplerHz, float initCodePhaseChips)
{
    m_carrFreqBasisHz = initDopplerHz;
    m_carrFreqHz = initDopplerHz;
    m_remCarrPhase = 0.0f;
    m_carrNco = 0.0f;
    m_carrNcoPrev = 0.0f;
    m_carrErrorCycles = 0.0f;
    m_carrErrorPrevCycles = 0.0f;

    m_fllNco = 0.0f;
    m_ipPrev = 0.0f;
    m_qpPrev = 0.0f;
    m_promptMagnitudeEma = 0.0f;
    m_blocksSinceInit = 0;

    m_remCodePhase = initCodePhaseChips;
    m_codeFreqBasisHz = GPS_CA_CODE_FREQUENCY_HZ;
    m_codeFreqHz = GPS_CA_CODE_FREQUENCY_HZ;
    if (m_configuration.acquisitionInput.samplingFrequencyHz > 0.0f)
    {
        m_codePhaseStep = m_codeFreqHz / m_configuration.acquisitionInput.samplingFrequencyHz;
    }
    m_codeNco = 0.0f;
    m_codeNcoPrev = 0.0f;
    m_codeErrorChips = 0.0f;
    m_codeErrorPrevChips = 0.0f;

    m_carrierLockEma = 0.0f;
    m_codeLockEma = 0.0f;

    resetAccumulation();
}

void Tracking::doWork(const ComplexFloatVector &input, int prn, ComplexFloatVector *output, uint32_t channelState)
{
    m_lastChannelState = channelState;

    m_earlyLatePromptGenTimeMs = 0.0f;
    m_numericOscillatorTimeMs = 0.0f;
    if (m_timingEnabled)
    {
        auto subStageT0 = std::chrono::high_resolution_clock::now();
        correlator(input, prn);
        auto subStageT1 = std::chrono::high_resolution_clock::now();
        m_accumulatorTimeMs = std::chrono::duration<float, std::milli>(subStageT1 - subStageT0).count();
    }
    else
    {
        correlator(input, prn);
        m_accumulatorTimeMs = 0.0f;
    }

    if (m_blocksSinceInit < m_fllPullInBlocks)
    {
        if (isPromptSignalReliable())
        {
            fllDiscriminator();
        }
        else
        {
            m_carrFreqHz = m_carrFreqBasisHz + m_fllNco;
        }
    }
    else
    {
        if (m_blocksSinceInit == m_fllPullInBlocks)
        {
            m_carrNco = m_fllNco;
            m_carrNcoPrev = m_fllNco;
            m_carrErrorPrevCycles = computeCostasPhaseError();
        }

        if (isPromptSignalReliable())
        {
            rateAidDiscriminator();
            freqDiscriminator();
        }
        else
        {
            m_carrFreqHz = m_carrFreqBasisHz + m_carrNco;
        }
    }
    codeDiscriminator();
    updateLockIndicators();
    m_blocksSinceInit++;

    if (output != nullptr)
    {
        output->emplace_back(m_Ip, m_Qp);
    }

    resetAccumulation();
}

TrackingOutput Tracking::getTrackingOutput(int prn) const
{
    TrackingOutput out{};
    out.structVersion = STRUCT_VERSION_2;
    out.prn = prn;
    out.carrierFreqHz = static_cast<double>(m_carrFreqHz);
    out.codeFreqHz = static_cast<double>(m_codeFreqHz);
    out.carrierErrorCycles = static_cast<double>(m_carrErrorCycles);
    out.codeErrorChips = static_cast<double>(m_codeErrorChips);
    out.Ie = static_cast<double>(m_Ie);
    out.Ip = static_cast<double>(m_Ip);
    out.Il = static_cast<double>(m_Il);
    out.Qe = static_cast<double>(m_Qe);
    out.Qp = static_cast<double>(m_Qp);
    out.Ql = static_cast<double>(m_Ql);
    out.channelState = m_lastChannelState;
    out.carrierLockIndicator = static_cast<double>(m_carrierLockEma);
    out.codeLockRatio = static_cast<double>(m_codeLockEma);
    out.correlatorTimeMs = static_cast<double>(m_accumulatorTimeMs);
    return out;
}

void Tracking::getSubStageTimings(float *earlyLatePromptGenMs, float *numericOscillatorMs, float *accumulatorMs) const
{
    *earlyLatePromptGenMs = m_earlyLatePromptGenTimeMs;
    *numericOscillatorMs = m_numericOscillatorTimeMs;
    *accumulatorMs = m_accumulatorTimeMs;
}

void Tracking::correlator(const ComplexFloatVector &input, int prn)
{
    const float samplingFreq = m_configuration.acquisitionInput.samplingFrequencyHz;
    if (samplingFreq <= 0.0f)
    {
        return;
    }

    const int svIndex = std::clamp(prn - 1, 0, GPS_CA_SV_COUNT - 1);
    const char *caCode = m_code.caCode[svIndex];

    const double phaseStepRad = 2.0 * M_PI * static_cast<double>(m_carrFreqHz) / samplingFreq;
    const std::complex<float> step = std::exp(IMAGINARY_UNIT * static_cast<float>(phaseStepRad));
    std::complex<float> phasor = std::exp(IMAGINARY_UNIT * m_remCarrPhase);

    const size_t length = std::min(input.size(), static_cast<size_t>(m_totalSamples));

    float ie = 0.0f;
    float qe = 0.0f;
    float ip = 0.0f;
    float qp = 0.0f;
    float il = 0.0f;
    float ql = 0.0f;

    for (size_t i = 0; i < length; i++)
    {
        const float phase = (static_cast<float>(i) * m_codePhaseStep) + m_remCodePhase;
        int rawPrompt = static_cast<int>(phase);
        rawPrompt -= (static_cast<float>(rawPrompt) > phase) ? 1 : 0;
        const float frac = phase - static_cast<float>(rawPrompt);

        int promptIndex = rawPrompt;
        while (promptIndex >= GPS_CA_CODE_LENGTH)
        {
            promptIndex -= GPS_CA_CODE_LENGTH;
        }
        while (promptIndex < 0)
        {
            promptIndex += GPS_CA_CODE_LENGTH;
        }

        int earlyIndex = (frac < 0.5f) ? promptIndex - 1 : promptIndex;
        earlyIndex += (earlyIndex < 0) ? GPS_CA_CODE_LENGTH : 0;
        int lateIndex = (frac < 0.5f) ? promptIndex : promptIndex + 1;
        lateIndex -= (lateIndex >= GPS_CA_CODE_LENGTH) ? GPS_CA_CODE_LENGTH : 0;

        const float earlyChip = caCode[earlyIndex];
        const float promptChip = caCode[promptIndex];
        const float lateChip = caCode[lateIndex];

        const std::complex<float> wipeoff = input[i] * std::conj(phasor);
        phasor *= step;

        const float re = wipeoff.real();
        const float im = wipeoff.imag();

        ie += re * earlyChip;
        qe += im * earlyChip;

        ip += re * promptChip;
        qp += im * promptChip;

        il += re * lateChip;
        ql += im * lateChip;
    }

    m_Ie = ie;
    m_Qe = qe;
    m_Ip = ip;
    m_Qp = qp;
    m_Il = il;
    m_Ql = ql;

    m_remCodePhase =
        static_cast<float>(std::fmod(m_remCodePhase + (static_cast<float>(m_totalSamples) * m_codePhaseStep), 1023.0));

    const double finalPhase = (phaseStepRad * m_totalSamples) + m_remCarrPhase;
    m_remCarrPhase = static_cast<float>(std::fmod(finalPhase, 2.0 * M_PI));
}

float Tracking::computeFllError(double ipPrev, double qpPrev, double ip, double qp)
{
    const double cross = (ipPrev * qp) - (ip * qpPrev);
    const double dot = (ipPrev * ip) + (qpPrev * qp);

    if (cross == 0.0 && dot == 0.0)
    {
        return 0.0f;
    }

    return static_cast<float>(std::atan2(cross, dot) / (2.0 * M_PI * GPS_CA_CODE_PERIOD_SEC));
}

void Tracking::fllDiscriminator()
{
    const float fllError = (m_blocksSinceInit > 0) ? computeFllError(m_ipPrev, m_qpPrev, m_Ip, m_Qp) : 0.0f;
    m_fllNco += fllError * m_fllGain;

    m_ipPrev = m_Ip;
    m_qpPrev = m_Qp;

    m_carrFreqHz = m_carrFreqBasisHz + m_fllNco;
}

bool Tracking::isPromptSignalReliable()
{
    const double currentMagnitude = std::sqrt((static_cast<double>(m_Ip) * m_Ip) + (static_cast<double>(m_Qp) * m_Qp));
    const bool reliable = (m_promptMagnitudeEma <= 0.0f) || (currentMagnitude >= 0.3 * m_promptMagnitudeEma);

    if (reliable)
    {
        m_promptMagnitudeEma = static_cast<float>((0.05 * currentMagnitude) + (0.95 * m_promptMagnitudeEma));
    }

    return reliable;
}

void Tracking::rateAidDiscriminator()
{
    const float bleed = m_rateAidGain * m_carrNco;
    m_carrFreqBasisHz += bleed;
    m_carrFreqBasisHz = std::clamp(m_carrFreqBasisHz, -15000.0f, 15000.0f);
    m_carrNco -= bleed;
    m_carrNcoPrev = m_carrNco;
}

float Tracking::computeCostasPhaseError() const
{
    if (m_Ip == 0.0f && m_Qp == 0.0f)
    {
        return 0.0f;
    }

    return static_cast<float>(std::atan2(2.0 * m_Qp * m_Ip, (m_Ip * m_Ip) - (m_Qp * m_Qp)) / (4.0 * M_PI));
}

void Tracking::freqDiscriminator()
{
    m_carrErrorCycles = computeCostasPhaseError();

    if (m_pllTau1 != 0.0f)
    {
        m_carrNco = m_carrNcoPrev + (m_pllTau2 / m_pllTau1) * (m_carrErrorCycles - m_carrErrorPrevCycles) +
            m_carrErrorCycles * static_cast<float>(GPS_CA_CODE_PERIOD_SEC / m_pllTau1);
    }

    m_carrNcoPrev = m_carrNco;
    m_carrErrorPrevCycles = m_carrErrorCycles;

    m_ipPrev = m_Ip;
    m_qpPrev = m_Qp;

    m_carrFreqHz = m_carrFreqBasisHz + m_carrNco;
}

void Tracking::codeDiscriminator()
{
    const double earlyCoff = std::sqrt((m_Ie * m_Ie) + (m_Qe * m_Qe));
    const double lateCoff = std::sqrt((m_Il * m_Il) + (m_Ql * m_Ql));
    const double denom = earlyCoff + lateCoff;

    if (denom != 0.0)
    {
        m_codeErrorChips = static_cast<float>(0.5 * (earlyCoff - lateCoff) / denom);
    }
    else
    {
        m_codeErrorChips = 0.0f;
    }

    if (m_dllTau1 != 0.0f)
    {
        m_codeNco = m_codeNcoPrev + ((m_dllTau2 / m_dllTau1) * (m_codeErrorChips - m_codeErrorPrevChips)) +
            (m_codeErrorChips * static_cast<float>(GPS_CA_CODE_PERIOD_SEC / m_dllTau1));
    }

    m_codeNcoPrev = m_codeNco;
    m_codeErrorPrevChips = m_codeErrorChips;

    m_codeFreqHz = m_codeFreqBasisHz + (m_carrFreqHz / GPS_L1_CARRIER_TO_CODE_RATIO) - m_codeNco;
    if (m_configuration.acquisitionInput.samplingFrequencyHz > 0.0f)
    {
        m_codePhaseStep = m_codeFreqHz / m_configuration.acquisitionInput.samplingFrequencyHz;
    }
}

void Tracking::updateLockIndicators()
{
    const double alpha = m_inputConfig.lockIndicatorEmaAlpha;

    const double instantCarrierLock = LockDetector::carrierLockIndicator(m_Ip, m_Qp);
    const double instantCodeLock = LockDetector::codeLockRatio(m_Ie, m_Ip, m_Il);

    m_carrierLockEma = static_cast<float>((alpha * instantCarrierLock) + ((1.0 - alpha) * m_carrierLockEma));
    m_codeLockEma = static_cast<float>((alpha * instantCodeLock) + ((1.0 - alpha) * m_codeLockEma));
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
