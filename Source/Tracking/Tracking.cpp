#include "Tracking.h"

#include <QElapsedTimer>
#include <QDebug>
#include "../Code/CACode.h"
#include "../Utils/Utils.h"
#include <cmath>

GPSOpenCl::Tracking::Tracking(std::vector<std::complex<double>> code,
                              std::vector<std::complex<double>> rawData,
                              QObject *parent) : QThread(parent)
{
    m_code = GPSOpenCl::CACode::calculateCACode(17);
    double initialCode = m_code.front();
    double lastCode = m_code.back(); 
    m_code.insert(m_code.begin(), lastCode);
    m_code.push_back(initialCode);

    m_totalSamples = 4096;
    m_rawData = rawData;

    m_remCodePhase = 0;
    m_carrFreqBasis = -289.0625;
    m_carrFreq = m_carrFreqBasis;
    m_remCarrPhase = 0;
    m_carrNco = 0;
    m_carrNcoPrev = 0;
    m_carrErrorPrev = 0;
    m_carrError = 0;
    m_codeNco = 0;
    m_codeNcoPrev = 0;
    m_codeError = 0;
    m_codeErrorPrev = 0;
    m_codeFreqBasis = 1023000;
    m_codeFreq = m_codeFreqBasis;

    m_Ie = 0;
    m_Ip = 0;
    m_Il = 0;
    m_Qe = 0;
    m_Qp = 0;
    m_Ql = 0;

    m_earlyCode.resize(4096);
    m_promptCode.resize(4096);
    m_lateCode.resize(4096);
    m_carrSig.resize(4096);

    m_logger = new Logger();

    calcLoopCoefficients(1.0, 0.7, 1.0, &m_dllTau1, &m_dllTau2);
    calcLoopCoefficients(6.5, 0.7, 0.25, &m_pllTau1, &m_pllTau2);
    m_codePhaseStep = m_codeFreq / 4096000;
}

GPSOpenCl::Tracking::~Tracking()
{
}

void GPSOpenCl::Tracking::run()
{
    // Get high resolution timer
    QElapsedTimer timer;
    int initOffset = 3777;
    for (int i = 0; i < 999; i++)
    {
        timer.start();

        m_codePhaseStep = m_codeFreq/4096000.0;
        m_totalSamples = std::ceil((1023 - m_remCodePhase) / m_codePhaseStep);

        std::vector<std::complex<double>> inputSignal;
        std::copy(m_rawData.begin() + initOffset, m_rawData.begin() + initOffset + m_totalSamples, std::back_inserter(inputSignal));
        
        earlyLatePromptGen();
        numericOscillator();
        accumulator(inputSignal);
        Ip.push_back(m_Ip);
        freqDiscriminator();
        codeDiscriminator();
        resetAccumulation();
        initOffset += m_totalSamples;

        qDebug() << "Tracking operation took" << timer.nsecsElapsed() << "nanoseconds";
    }
}

void GPSOpenCl::Tracking::calcLoopCoefficients(double noiseBandWidth,
                                               double dampingRatio,
                                               double gain,
                                               double *tau1,
                                               double *tau2)
{
    double naturalFreq = (noiseBandWidth * 8.0 * dampingRatio) / (4 * std::pow(dampingRatio, 2.0) + 1);

    *tau1 = gain / (naturalFreq * naturalFreq);
    *tau2 = 2.0 * dampingRatio / naturalFreq;
}

void GPSOpenCl::Tracking::earlyLatePromptGen()
{
    double phaseStep = m_remCodePhase;
    for (int i = 0; i < m_totalSamples; i++)
    {
        phaseStep = i * m_codePhaseStep + m_remCodePhase;
        m_earlyCode[i] = m_code.at(static_cast<int>(std::ceil(phaseStep - 0.5)));
        m_promptCode[i] = m_code.at(static_cast<int>(std::ceil(phaseStep)));
        m_lateCode[i] = m_code.at(static_cast<int>(std::ceil(phaseStep + 0.5)));
    }
    m_remCodePhase = (phaseStep + m_codePhaseStep) - 1023.0;
}

void GPSOpenCl::Tracking::numericOscillator()
{
    const std::complex<double> i(0, 1);
    double phase = 0;
    for (int sample = 0; sample < m_totalSamples; sample++)
    {
        double sampDouble = static_cast<double>(sample);
        phase = (2 * M_PI * m_carrFreq * sampDouble / 4096000.0) + m_remCarrPhase;
        m_carrSig[sample] = std::exp(i *phase);
    }
    phase = (2 * M_PI * m_carrFreq * m_totalSamples / 4096000.0) + m_remCarrPhase;
    m_remCarrPhase = std::fmod(phase, (2 * M_PI));
}

void GPSOpenCl::Tracking::accumulator(std::vector<std::complex<double>> input)
{
    std::complex<double> freqMult;
    for (int i = 0; i < m_totalSamples; i++)
    {
        freqMult = m_carrSig.at(i) * input.at(i);
        m_Ie += std::imag(freqMult * m_earlyCode.at(i));
        m_Qe += std::real(freqMult * m_earlyCode.at(i));
        m_Ip += std::imag(freqMult * m_promptCode.at(i));
        m_Qp += std::real(freqMult * m_promptCode.at(i));
        m_Il += std::imag(freqMult * m_lateCode.at(i));
        m_Ql += std::real(freqMult * m_lateCode.at(i));
    }
}

void GPSOpenCl::Tracking::freqDiscriminator()
{
    m_carrError = std::atan(m_Qp / m_Ip) / (2.0 * M_PI);
    m_carrNco = m_carrNcoPrev + (m_pllTau2 / m_pllTau1) * (m_carrError - m_carrErrorPrev) + m_carrError * (0.001 / m_pllTau1);
    m_carrNcoPrev = m_carrNco;
    m_carrErrorPrev = m_carrError;
    m_carrFreq = m_carrFreqBasis + m_carrNco;
}

void GPSOpenCl::Tracking::codeDiscriminator()
{
    double earlyCoff = std::sqrt(m_Ie * m_Ie + m_Qe * m_Qe);
    double lateCoff = std::sqrt(m_Il * m_Il + m_Ql * m_Ql);
    m_codeError = (earlyCoff - lateCoff) / (earlyCoff + lateCoff); 
    m_codeNco = m_codeNcoPrev + (m_dllTau2 / m_dllTau1) * (m_codeError - m_codeErrorPrev) + m_codeError * (0.001 / m_dllTau1);
    m_codeNcoPrev = m_codeNco;
    m_codeErrorPrev = m_codeError;
    m_codeFreq = m_codeFreqBasis - m_codeNco;
}

void GPSOpenCl::Tracking::resetAccumulation()
{
    m_Ie = 0;
    m_Ip = 0;
    m_Il = 0;
    m_Qe = 0;
    m_Qp = 0;
    m_Ql = 0;
}