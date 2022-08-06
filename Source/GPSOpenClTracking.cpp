#include "GPSOpenClTracking.h"

using namespace GPSOpenCl;

Tracking::Tracking(Settings::Configuration conf)
    : m_configuration(conf), m_Ie(0.0f), m_Ip(0.0f), m_Il(0.0f), m_Qe(0.0f), m_Qp(0.0f), m_Ql(0.0f)
{
    m_gpu = new Compute();

    m_code.setConfiguration(m_configuration);

    m_totalSamples = m_configuration.rawDataSettings.numberOfSamplesPerCode;

    m_carrSig.resize(m_totalSamples);
    m_earlyCode.resize(m_totalSamples);
    m_promptCode.resize(m_totalSamples);
    m_lateCode.resize(m_totalSamples);
}

Tracking::~Tracking()
{
}

void Tracking::doWork(const ComplexFloatVector input, int prn, ComplexFloatVector *output)
{
    earlyLatePromptGen(prn);
    numericOscillator();
    // accumulator(input);
    freqDiscriminator();
    codeDiscriminator();
    resetAccumulation();
}

void Tracking::earlyLatePromptGen(int prn)
{
    int earlyIndex = 0;
    int promptIndex = 0;
    int lateIndex = 0;

    double phaseStep = m_remCodePhase;
    for (int i = 0; i < m_totalSamples; i++)
    {
        phaseStep = i * m_codePhaseStep + m_remCodePhase;

        earlyIndex = static_cast<int>(std::ceil(phaseStep - 0.5));
        promptIndex = static_cast<int>(std::ceil(phaseStep));
        lateIndex = static_cast<int>(std::ceil(phaseStep + 0.5));

        m_earlyCode[i] = m_code.m_caCode[prn][earlyIndex];
        m_promptCode[i] = m_code.m_caCode[prn][promptIndex];
        m_lateCode[i] = m_code.m_caCode[prn][lateIndex];
    }
    m_remCodePhase = (phaseStep + m_codePhaseStep) - 1023.0;
}

void Tracking::numericOscillator()
{
    float phase = 0;

    for (int sample = 0; sample < m_totalSamples; sample++)
    {
        double sampDouble = static_cast<double>(sample);
        phase =
            (2 * M_PI * m_carrFreq * sampDouble / m_configuration.rawDataSettings.samplingFrequency) + m_remCarrPhase;
        m_carrSig[sample] = std::exp(IMAGINARY_UNIT * phase);
    }

    phase =
        (2 * M_PI * m_carrFreq * m_totalSamples / m_configuration.rawDataSettings.samplingFrequency) + m_remCarrPhase;
    m_remCarrPhase = std::fmod(phase, (2 * M_PI));
}

void Tracking::freqDiscriminator()
{
    m_carrError = std::atan(m_Qp / m_Ip) / (2.0 * M_PI);

    m_carrNco = m_carrNcoPrev + (m_pllTau2 / m_pllTau1) * (m_carrError - m_carrErrorPrev) +
                m_carrError * (GPS_CA_CODE_PERIOD_SEC / m_pllTau1);

    m_carrNcoPrev = m_carrNco;
    m_carrErrorPrev = m_carrError;

    m_carrFreq = m_carrFreqBasis + m_carrNco;
}

void Tracking::codeDiscriminator()
{
    double earlyCoff = std::sqrt(m_Ie * m_Ie + m_Qe * m_Qe);
    double lateCoff = std::sqrt(m_Il * m_Il + m_Ql * m_Ql);

    m_codeError = (earlyCoff - lateCoff) / (earlyCoff + lateCoff);

    m_codeNco = m_codeNcoPrev + (m_dllTau2 / m_dllTau1) * (m_codeError - m_codeErrorPrev) +
                m_codeError * (GPS_CA_CODE_PERIOD_SEC / m_dllTau1);

    m_codeNcoPrev = m_codeNco;
    m_codeErrorPrev = m_codeError;

    m_codeFreq = m_codeFreqBasis - m_codeNco;
}

void Tracking::resetAccumulation()
{
    m_Ie = 0;
    m_Ip = 0;
    m_Il = 0;
    m_Qe = 0;
    m_Qp = 0;
    m_Ql = 0;
}