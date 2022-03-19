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
        std::vector<std::complex<double>> m_code;
        std::vector<std::complex<double>> m_rawData;
    };
}

#endif // ! INCLUDED_TRACKING_H