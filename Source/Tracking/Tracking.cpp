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
    m_code.insert(m_code.begin(), m_code.back());
    m_code.push_back(m_code.front());

    m_totalSamples = 4096;
    m_rawData = rawData;

    m_remCodePhase = 0;
    m_carrFreq = -289.0625;
    m_remCarrPhase = 0;
    m_carrNco = 0;
    m_carrNcoPrev = 0;
    m_carrErrorPrev = 0;
    m_carrError = 0;
    m_codeNco = 0;
    m_codeNcoPrev = 0;
    m_codeError = 0;
    m_codeErrorPrev = 0;
    m_codeFreq = 1023000;

    m_Ie = 0;
    m_Ip = 0;
    m_Il = 0;
    m_Qe = 0;
    m_Qp = 0;
    m_Ql = 0;

    m_earlyCode.resize(4096);
    m_promptCode.resize(4096);
    m_lateCode.resize(4096);

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
    std::vector<std::complex<double>> inputSignal;
    std::copy(m_rawData.begin() + 3777, m_rawData.begin() + 3777 + m_totalSamples, std::back_inserter(inputSignal));
    for (int i = 0; i < 10; i++)
    {
        timer.start();

        earlyLatePromptGen();
        numericOscillator();
        accumulator(inputSignal);
        freqDiscriminator();
        codeDiscriminator();
        std::copy(m_rawData.begin() + 3777, m_rawData.begin() + 3777 + m_totalSamples, inputSignal.begin());

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
    int indexEarly = 0;
    int indexPrompt = 0;
    int indexLate = 0;
    m_codePhaseStep = m_codeFreq / 4096000;
    for (int i = 0; i < 4096; i++)
    {
        indexEarly = static_cast<int>(std::ceil(m_remCodePhase - 0.5 + i * m_codePhaseStep));
        indexPrompt = static_cast<int>(std::ceil(m_remCodePhase + i * m_codePhaseStep));
        indexLate = static_cast<int>(std::ceil(m_remCodePhase + 0.5 + i * m_codePhaseStep));
        m_earlyCode[i] = m_code.at(indexEarly);
        m_promptCode[i] = m_code.at(indexPrompt);
        m_lateCode[i] = m_code.at(indexLate);
    }
    m_remCodePhase = (m_totalSamples * m_codePhaseStep) - 1023.0;
}

void GPSOpenCl::Tracking::numericOscillator()
{
    m_carrSig = GPSOpenCl::Utils::exp(4096, m_carrFreq, 4096000.0, m_remCarrPhase);
    m_remCarrPhase = std::remainder((2.0 * M_PI * m_carrFreq * 0.001) + m_remCarrPhase, (2 * M_PI));
}

void GPSOpenCl::Tracking::accumulator(std::vector<std::complex<double>> input)
{
    std::complex<double> freqMult;
    for (int i = 0; i < m_totalSamples; i++)
    {
        freqMult = m_carrSig.at(i) * input.at(i);
        m_Ie += std::real(freqMult * m_earlyCode.at(i));
        m_Qe += std::imag(freqMult * m_earlyCode.at(i));
        m_Ip += std::real(freqMult * m_promptCode.at(i));
        m_Qp += std::imag(freqMult * m_promptCode.at(i));
        m_Il += std::real(freqMult * m_lateCode.at(i));
        m_Ql += std::imag(freqMult * m_lateCode.at(i));
    }
}

void GPSOpenCl::Tracking::freqDiscriminator()
{
    double carrError = std::atan(m_Qp / m_Ip) / (2.0 * M_PI);
    m_carrFreq = m_carrFreq + (m_pllTau1 * carrError);
    m_carrNco = m_carrNcoPrev + (m_pllTau2 / m_pllTau1) * (m_carrError - m_carrErrorPrev) + m_carrError * (0.001 / m_pllTau1);
    m_carrNcoPrev = m_carrNco;
    m_carrErrorPrev = m_carrError;
}

void GPSOpenCl::Tracking::codeDiscriminator()
{
    m_codeError = (std::sqrt(m_Ie * m_Ie + m_Qe * m_Qe) - std::sqrt(m_Il * m_Il + m_Ql * m_Ql)) / (std::sqrt(m_Ie * m_Ie + m_Qe * m_Qe) + sqrt(m_Il * m_Il + m_Ql * m_Ql));
    m_codeNco = m_codeNcoPrev + (m_dllTau2 / m_dllTau1) * (m_codeError - m_codeErrorPrev) + m_codeError * (0.001 / m_dllTau1);
    m_codeNcoPrev = m_codeNco;
    m_codeErrorPrev = m_codeError;
    m_codeFreq = m_codeFreq + -m_codeNco;
}