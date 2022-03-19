#ifndef INCLUDED_CA_CODE_H
#define INCLUDED_CA_CODE_H

#include <complex>
#include <QDebug>
#include "../Logger/Logger.h"

namespace GPSOpenCl
{
    class CACode
    {
    public:
        CACode();
        ~CACode();
        void createCACodeTable();
        std::vector<std::vector<std::complex<double>>> m_code;

    private:
        double *calculateCACode(int prn);
        double m_samplingTime;
        double m_codePeriodTime;
        int m_codeResampledLength;
    };
}

#endif //! INCLUDED_CA_CODE_H