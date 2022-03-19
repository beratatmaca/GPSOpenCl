#include "Tracking.h"

#include <QElapsedTimer>
#include <QDebug>

GPSOpenCl::Tracking::Tracking(std::vector<std::complex<double>> code, 
                                    std::vector<std::complex<double>> rawData,
                                    QObject *parent) : QThread(parent)
{
    m_code = code;
    m_rawData = rawData;   

    calcLoopCoefficients(1.0, 0.7, 1.0, &m_dllTau1, &m_dllTau2); 
    calcLoopCoefficients(6.5, 0.7, 0.25, &m_pllTau1, &m_pllTau2); 
}

GPSOpenCl::Tracking::~Tracking()
{

}

void GPSOpenCl::Tracking::run()
{
    // Get high resolution timer
    QElapsedTimer timer;
    timer.start();

    /*Code*/

    qDebug() << "Tracking operation took" << timer.nsecsElapsed() << "nanoseconds";
    qDebug() << m_dllTau1;
    qDebug() << m_dllTau2;
    qDebug() << m_pllTau1;
    qDebug() << m_pllTau2;
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
