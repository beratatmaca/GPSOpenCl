#ifndef INCLUDED_ACQUISITION_H
#define INCLUDED_ACQUISITION_H

#include <QThread>
#include <complex>
#include "../Logger/Logger.h"

namespace GPSOpenCl
{
    class Acquisition : public QThread
    {
    Q_OBJECT
    public:
        Acquisition(double* code, std::complex<double> rawData, QObject *parent = 0);
        ~Acquisition();
        void run();
    private:
        Logger* m_logger;
        double m_code[4096];
        double freqList[29];
        int m_freqBins;
        std::complex<double> m_rawData[4096];
    };
}

#endif // ! INCLUDED_ACQUISITION_H