#ifndef INCLUDED_ACQUISITION_H
#define INCLUDED_ACQUISITION_H

#include <QThread>
#include <QMutex>
#include <complex>
#include "../Logger/Logger.h"
#include "../Utils/FftUtils.h"
#include "../Utils/Utils.h"

namespace GPSOpenCl
{
    class Acquisition : public QThread
    {
        Q_OBJECT
    public:
        Acquisition(std::vector<std::complex<double>> code,
                    std::vector<std::complex<double>> rawData,
                    double threshold,
                    QObject *parent = 0);
        ~Acquisition();
        void run();
    signals:
        void acquisitionSuccessful(double peakFreq, double peakCodeIdx, double snr);
        void acquisitionFailed();

    private:
        std::vector<std::vector<double>> correlator();
        void checkResults(std::vector<std::vector<double>> corrResult);

        Logger *m_logger;
        FftUtils *m_fftUtils;

        std::vector<std::complex<double>> m_code;
        std::vector<std::complex<double>> m_rawData;
        double freqList[29];
        int m_freqBins;
        double m_duration_ms;

        double m_snr;
        double m_snrThreshold;
        double m_peakFreq;
        double m_peakVal;
        double m_peakIndex;
    };
}

#endif // ! INCLUDED_ACQUISITION_H