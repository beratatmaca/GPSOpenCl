#ifndef INCLUDED_FFTUTILS_H
#define INCLUDED_FFTUTILS_H

#include "clFFT.h" // for clFFT
#include <complex> // for std::complex
#include <QDebug> // for qDebug()

namespace GPSOpenCl
{
    class FftUtils
    {
    public:
        FftUtils(int length);
        ~FftUtils();
        std::complex<double> *fftReal(double input[]);
        std::complex<double> *fftComplex(std::complex<double> input[]);

    private:
        cl_context m_ctx;
        clfftPlanHandle m_planHandle;
        cl_command_queue m_queue;
        int m_length;

        cl_float *m_inReal;
        cl_float *m_inImag;
        cl_float *m_outReal;
        cl_float *m_outImag;

        cl_mem m_tmpBuffer;
        cl_mem m_buffersIn[2] = {0, 0};
        cl_mem m_buffersOut[2] = {0, 0};
    };
}

#endif // ! INCLUDED_FFTUTILS_H