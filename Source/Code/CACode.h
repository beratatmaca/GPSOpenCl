#ifndef INCLUDED_CA_CODE_H
#define INCLUDED_CA_CODE_H

namespace GPSOpenCl
{
    class CACode
    {
    public:
        CACode();
        ~CACode();
        void createCACodeTable();
        double m_data[32][16384];
    private:
        double* calculateCACode(int prn);
        double m_samplingTime;
        double m_codePeriodTime;
        int m_codeResampledLength;
    };    
}

#endif //! INCLUDED_CA_CODE_H