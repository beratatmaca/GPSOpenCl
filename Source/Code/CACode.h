#ifndef INCLUDED_CA_CODE_H
#define INCLUDED_CA_CODE_H

#include <complex>

namespace GPSOpenCl
{
    class CACode
    {
    public:
        CACode();
        ~CACode();
        void createCACodeTable();
        static std::complex<double>* conjFFTcode(double caCode[], int codeLength);
        double m_code[32][4096];
    private:
        double* calculateCACode(int prn);
        double m_samplingTime;
        double m_codePeriodTime;
        int m_codeResampledLength;
    };    
}

#endif //! INCLUDED_CA_CODE_H