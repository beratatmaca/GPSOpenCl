#include "FftUtils.h"

GPSOpenCl::FftUtils::FftUtils(int length)
{
    m_length = length;
    m_queue = 0;
    m_tmpBuffer = 0;
    m_ctx = 0;

    cl_int err;
    cl_platform_id platform = 0;
    cl_device_id device = 0;
    clfftSetupData fftSetup;
    clfftDim dim = CLFFT_1D;
    size_t clLengths[1] = {static_cast<size_t>(m_length)};

    err = clGetPlatformIDs(1, &platform, NULL);
    err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, NULL);
    m_ctx = clCreateContext(NULL, 1, &device, NULL, NULL, &err);
    m_queue = clCreateCommandQueueWithProperties(m_ctx, device, 0, &err);
    err = clfftInitSetupData(&fftSetup);
    err = clfftSetup(&fftSetup);
    err = clfftCreateDefaultPlan(&m_planHandle, m_ctx, dim, clLengths);
    err = clfftSetPlanPrecision(m_planHandle, CLFFT_SINGLE);
    err = clfftSetLayout(m_planHandle, CLFFT_COMPLEX_PLANAR, CLFFT_COMPLEX_PLANAR);
    err = clfftSetResultLocation(m_planHandle, CLFFT_OUTOFPLACE);
    err = clfftBakePlan(m_planHandle, 1, &m_queue, NULL, NULL);

    size_t tmpBufferSize = 0;
    int status = 0;
    status = clfftGetTmpBufSize(m_planHandle, &tmpBufferSize);
    if ((status == 0) && (tmpBufferSize > 0))
    {
        m_tmpBuffer = clCreateBuffer(m_ctx, CL_MEM_READ_WRITE, tmpBufferSize, 0, &err);
        if (err != CL_SUCCESS)
        {
            qDebug() << "Error with tmpBuffer clCreateBuffer";
        }
    }

    // Real and Imaginary arrays
    m_inReal = (cl_float *)malloc(m_length * sizeof(cl_float));
    m_inImag = (cl_float *)malloc(m_length * sizeof(cl_float));
    m_outReal = (cl_float *)malloc(m_length * sizeof(cl_float));
    m_outImag = (cl_float *)malloc(m_length * sizeof(cl_float));

    // Prepare OpenCL memory objects : create buffer for input
    m_buffersIn[0] = clCreateBuffer(m_ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, m_length * sizeof(cl_float), m_inReal, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[0] clCreateBuffer";
    }
    m_buffersIn[1] = clCreateBuffer(m_ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, m_length * sizeof(cl_float), m_inImag, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[1] clCreateBuffer";
    }
    // Prepare OpenCL memory objects : create buffer for output
    m_buffersOut[0] = clCreateBuffer(m_ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, m_length * sizeof(cl_float), m_outReal, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersOut[1] clCreateBuffer";
    }
    m_buffersOut[1] = clCreateBuffer(m_ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, m_length * sizeof(cl_float), m_outImag, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersOut[1] clCreateBuffer";
    }
}

GPSOpenCl::FftUtils::~FftUtils()
{
    // Release OpenCL memory objects
    clReleaseMemObject(m_buffersIn[0]);
    clReleaseMemObject(m_buffersIn[1]);
    clReleaseMemObject(m_buffersOut[0]);
    clReleaseMemObject(m_buffersOut[1]);
    clReleaseMemObject(m_tmpBuffer);

    delete m_inReal;
    delete m_inImag;
    delete m_outReal;
    delete m_outImag;

    clfftDestroyPlan(&m_planHandle);
    clfftTeardown();
    clReleaseCommandQueue(m_queue);
    clReleaseContext(m_ctx);
}

std::vector<std::complex<double>> GPSOpenCl::FftUtils::fftComplex(std::vector<std::complex<double>> input)
{
    cl_int err;
    std::vector<std::complex<double>> retVal;

    for (int j = 0; j < m_length; j++)
    {
        m_inReal[j] = static_cast<float>(std::real(input.at(j)));
        m_inImag[j] = static_cast<float>(std::imag(input.at(j)));
        m_outReal[j] = 0.0;
        m_outImag[j] = 0.0;
    }

    err = clEnqueueWriteBuffer(m_queue, m_buffersIn[0], CL_TRUE, 0, m_length * sizeof(float), m_inReal, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[0] clEnqueueWriteBuffer";
    }
    err = clEnqueueWriteBuffer(m_queue, m_buffersIn[1], CL_TRUE, 0, m_length * sizeof(float), m_inImag, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[1] clEnqueueWriteBuffer\n";
    }
    // Execute the plan
    err = clfftEnqueueTransform(m_planHandle, CLFFT_FORWARD, 1, &m_queue, 0, NULL, NULL,
                                m_buffersIn, m_buffersOut, m_tmpBuffer);
    err = clFinish(m_queue);
    float outRealCPU[m_length] = {0};
    float outImagCPU[m_length] = {0};
    err = clEnqueueReadBuffer(m_queue, m_buffersOut[0], CL_TRUE, 0, m_length * sizeof(float), &outRealCPU[0],
                              0, NULL, NULL);
    err = clEnqueueReadBuffer(m_queue, m_buffersOut[1], CL_TRUE, 0, m_length * sizeof(float), &outImagCPU[0],
                              0, NULL, NULL);

    for (int k = 0; k < m_length; k++)
    {
        retVal.push_back(std::complex<double>(outRealCPU[k], outImagCPU[k]));
    }
    return retVal;
}

std::vector<std::complex<double>> GPSOpenCl::FftUtils::ifftComplex(std::vector<std::complex<double>> input)
{
    cl_int err;
    std::vector<std::complex<double>> retVal;

    for (int j = 0; j < m_length; j++)
    {
        m_inReal[j] = static_cast<float>(std::real(input.at(j)));
        m_inImag[j] = static_cast<float>(std::imag(input.at(j)));
        m_outReal[j] = 0.0;
        m_outImag[j] = 0.0;
    }

    err = clEnqueueWriteBuffer(m_queue, m_buffersIn[0], CL_TRUE, 0, m_length * sizeof(float), m_inReal, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[0] clEnqueueWriteBuffer";
    }
    err = clEnqueueWriteBuffer(m_queue, m_buffersIn[1], CL_TRUE, 0, m_length * sizeof(float), m_inImag, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[1] clEnqueueWriteBuffer\n";
    }
    // Execute the plan
    err = clfftEnqueueTransform(m_planHandle, CLFFT_BACKWARD, 1, &m_queue, 0, NULL, NULL,
                                m_buffersIn, m_buffersOut, m_tmpBuffer);
    err = clFinish(m_queue);
    float outRealCPU[m_length] = {0};
    float outImagCPU[m_length] = {0};
    err = clEnqueueReadBuffer(m_queue, m_buffersOut[0], CL_TRUE, 0, m_length * sizeof(float), &outRealCPU[0],
                              0, NULL, NULL);
    err = clEnqueueReadBuffer(m_queue, m_buffersOut[1], CL_TRUE, 0, m_length * sizeof(float), &outImagCPU[0],
                              0, NULL, NULL);

    for (int k = 0; k < m_length; k++)
    {
        retVal.push_back(std::complex<double>(outRealCPU[k], outImagCPU[k]));
    }
    return retVal;    
}