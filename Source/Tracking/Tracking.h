#ifndef INCLUDED_TRACKING_H
#define INCLUDED_TRACKING_H

#include <QThread>
#include <QMutex>
#include <complex>

namespace GPSOpenCl
{
    class Tracking : public QThread
    {
        Q_OBJECT
    public:
        Tracking(std::vector<std::complex<double>> code,
                 std::vector<std::complex<double>> rawData,
                 QObject *parent = 0);
        ~Tracking();
        void run();
    private:        
        void earlyLatePromptGen();
        void numericOscillator();
        void accumulator(std::vector<std::complex<double>> input);
        void freqDiscriminator();
        void codeDiscriminator();
        void calcLoopCoefficients(double noiseBandWidth,
                                  double dampingRatio,
                                  double gain,
                                  double *tau1,
                                  double *tau2);
        
        std::vector<double> m_code;
        std::vector<double> m_earlyCode;
        std::vector<double> m_promptCode;
        std::vector<double> m_lateCode;
        std::vector<std::complex<double>> m_rawData;

        int m_totalSamples;

        double m_dllTau1;
        double m_dllTau2;
        double m_codeFreq;
        double m_codePhaseStep;
        double m_remCodePhase;
        double m_codeNco;
        double m_codeNcoPrev;
        double m_codeError;
        double m_codeErrorPrev;

        std::vector<std::complex<double>> m_carrSig;
        double m_pllTau1;
        double m_pllTau2;
        double m_carrFreq;
        double m_remCarrPhase;
        double m_carrNco;
        double m_carrNcoPrev;
        double m_carrError;
        double m_carrErrorPrev;

        double m_Ie;
        double m_Ip;
        double m_Il;
        double m_Qe;
        double m_Qp;
        double m_Ql;
    };
}

#endif // ! INCLUDED_TRACKING_H