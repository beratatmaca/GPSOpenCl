#ifndef INCLUDED_ACQUISITION_H
#define INCLUDED_ACQUISITION_H

#include <QThread>

namespace GPSOpenCl
{
    class Acquisition : public QThread
    {
        Acquisition(int prn);
        ~Acquisition();
        void run();
    private:
        int m_prn;
    };
}

#endif // ! INCLUDED_ACQUISITION_H