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
        void calcLoopCoefficients(double noiseBandWidth,
                                  double dampingRatio,
                                  double gain,
                                  double *tau1,
                                  double *tau2);
        std::vector<std::complex<double>> m_code;
        std::vector<std::complex<double>> m_rawData;

        double m_dllTau1;
        double m_dllTau2;
        double m_pllTau1;
        double m_pllTau2;
    };
}

#endif // ! INCLUDED_TRACKING_H