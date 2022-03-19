#include "Tracking.h"

#include <QElapsedTimer>
#include <QDebug>

GPSOpenCl::Tracking::Tracking(std::vector<std::complex<double>> code, 
                                    std::vector<std::complex<double>> rawData,
                                    QObject *parent) : QThread(parent)
{
    m_code = code;
    m_rawData = rawData;    
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
}
