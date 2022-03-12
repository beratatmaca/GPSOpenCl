#include "Utils.h"

#include <clFFT.h> // for fft

GPSOpenCl::Utils::Utils()
{
}

GPSOpenCl::Utils::~Utils()
{
}

std::complex<double> *GPSOpenCl::Utils::fftReal(double input[], int length)
{
    std::complex<double> *retVal = (std::complex<double> *)malloc(sizeof(std::complex<double>) * length);
    // OpenCL variables
    cl_int err;
    cl_platform_id platform = 0;
    cl_device_id device = 0;
    cl_context ctx = 0;
    cl_command_queue queue = 0;

    // Setup OpenCL environment
    err = clGetPlatformIDs(1, &platform, NULL);
    err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, NULL);

    // Create an OpenCL context
    ctx = clCreateContext(NULL, 1, &device, NULL, NULL, &err);

    // Create a command queue
    queue = clCreateCommandQueueWithProperties(ctx, device, 0, &err);

    // Setup clFFT
    clfftSetupData fftSetup;
    err = clfftInitSetupData(&fftSetup);
    err = clfftSetup(&fftSetup);

    // FFT library realted declarations
    clfftPlanHandle planHandle;
    clfftDim dim = CLFFT_1D;
    size_t clLengths[1] = {static_cast<size_t>(length)};

    // Create a default plan for a complex FFT
    err = clfftCreateDefaultPlan(&planHandle, ctx, dim, clLengths);

    // Set plan parameters
    err = clfftSetPlanPrecision(planHandle, CLFFT_SINGLE);
    err = clfftSetLayout(planHandle, CLFFT_COMPLEX_PLANAR, CLFFT_COMPLEX_PLANAR);
    err = clfftSetResultLocation(planHandle, CLFFT_OUTOFPLACE);

    // Real and Imaginary arrays
    cl_float *inReal = (cl_float *)malloc(length * sizeof(cl_float));
    cl_float *inImag = (cl_float *)malloc(length * sizeof(cl_float));
    cl_float *outReal = (cl_float *)malloc(length * sizeof(cl_float));
    cl_float *outImag = (cl_float *)malloc(length * sizeof(cl_float));

    // Input and Output buffer
    cl_mem buffersIn[2] = {0, 0};
    cl_mem buffersOut[2] = {0, 0};

    // Bake the plan
    err = clfftBakePlan(planHandle, 1, &queue, NULL, NULL);
    // Size of temp buffer
    size_t tmpBufferSize = 0;
    int status = 0;
    // Create temporary buffer
    status = clfftGetTmpBufSize(planHandle, &tmpBufferSize);
    // Temporary buffer
    cl_mem tmpBuffer = 0;
    if ((status == 0) && (tmpBufferSize > 0))
    {
        tmpBuffer = clCreateBuffer(ctx, CL_MEM_READ_WRITE, tmpBufferSize, 0, &err);
        if (err != CL_SUCCESS)
        {
            qDebug() << "Error with tmpBuffer clCreateBuffer";
        }
    }
    float outRealCPU[length] = {0};
    float outImagCPU[length] = {0};

    for (int j = 0; j < length; j++)
    {
        inReal[j] = static_cast<float>(input[j]);
        inImag[j] = 0.0;
        outReal[j] = 0.0;
        outImag[j] = 0.0;
    }
    // Prepare OpenCL memory objects : create buffer for input
    buffersIn[0] = clCreateBuffer(ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, length * sizeof(cl_float), inReal, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[0] clCreateBuffer";
    }

    err = clEnqueueWriteBuffer(queue, buffersIn[0], CL_TRUE, 0, length * sizeof(float), inReal, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[0] clEnqueueWriteBuffer";
    }
    // Prepare OpenCL memory objects : create buffer for input
    buffersIn[1] = clCreateBuffer(ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, length * sizeof(cl_float), inImag, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[1] clCreateBuffer";
    }

    // Enqueue write tab array into buffersIn[1]
    err = clEnqueueWriteBuffer(queue, buffersIn[1], CL_TRUE, 0, length * sizeof(float), inImag, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[1] clEnqueueWriteBuffer\n";
    }

    // Prepare OpenCL memory objects : create buffer for output
    buffersOut[0] = clCreateBuffer(ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, length * sizeof(cl_float), outReal, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersOut[1] clCreateBuffer";
    }

    buffersOut[1] = clCreateBuffer(ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, length * sizeof(cl_float), outImag, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersOut[1] clCreateBuffer";
    }

    // Execute the plan
    err = clfftEnqueueTransform(planHandle, CLFFT_FORWARD, 1, &queue, 0, NULL, NULL,
                                buffersIn, buffersOut, tmpBuffer);
    // Wait for calculations to be finished
    err = clFinish(queue);
    // Fetch results of calculations : Real and Imaginary
    err = clEnqueueReadBuffer(queue, buffersOut[0], CL_TRUE, 0, length * sizeof(float), &outRealCPU[0],
                              0, NULL, NULL);
    err = clEnqueueReadBuffer(queue, buffersOut[1], CL_TRUE, 0, length * sizeof(float), &outImagCPU[0],
                              0, NULL, NULL);

    for (int k = 0; k < length; k++)
    {
        retVal[k] = std::complex<double>(outRealCPU[k], outImagCPU[k]);
    }

    // Release OpenCL memory objects
    clReleaseMemObject(buffersIn[0]);
    clReleaseMemObject(buffersIn[1]);
    clReleaseMemObject(buffersOut[0]);
    clReleaseMemObject(buffersOut[1]);
    clReleaseMemObject(tmpBuffer);

    delete inReal;
    delete inImag;
    delete outReal;
    delete outImag;

    // Release the plan
    err = clfftDestroyPlan(&planHandle);

    // Release clFFT library
    clfftTeardown();

    // Release OpenCL working objects
    clReleaseCommandQueue(queue);
    clReleaseContext(ctx);
    return retVal;
}

std::complex<double> *GPSOpenCl::Utils::fftComplex(std::complex<double> input[], int length)
{
    std::complex<double> *retVal = (std::complex<double> *)malloc(sizeof(std::complex<double>) * length);
    // OpenCL variables
    cl_int err;
    cl_platform_id platform = 0;
    cl_device_id device = 0;
    cl_context ctx = 0;
    cl_command_queue queue = 0;

    // Setup OpenCL environment
    err = clGetPlatformIDs(1, &platform, NULL);
    err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, NULL);

    // Create an OpenCL context
    ctx = clCreateContext(NULL, 1, &device, NULL, NULL, &err);

    // Create a command queue
    queue = clCreateCommandQueueWithProperties(ctx, device, 0, &err);

    // Setup clFFT
    clfftSetupData fftSetup;
    err = clfftInitSetupData(&fftSetup);
    err = clfftSetup(&fftSetup);

    // FFT library realted declarations
    clfftPlanHandle planHandle;
    clfftDim dim = CLFFT_1D;
    size_t clLengths[1] = {static_cast<size_t>(length)};

    // Create a default plan for a complex FFT
    err = clfftCreateDefaultPlan(&planHandle, ctx, dim, clLengths);

    // Set plan parameters
    err = clfftSetPlanPrecision(planHandle, CLFFT_SINGLE);
    err = clfftSetLayout(planHandle, CLFFT_COMPLEX_PLANAR, CLFFT_COMPLEX_PLANAR);
    err = clfftSetResultLocation(planHandle, CLFFT_OUTOFPLACE);

    // Real and Imaginary arrays
    cl_float *inReal = (cl_float *)malloc(length * sizeof(cl_float));
    cl_float *inImag = (cl_float *)malloc(length * sizeof(cl_float));
    cl_float *outReal = (cl_float *)malloc(length * sizeof(cl_float));
    cl_float *outImag = (cl_float *)malloc(length * sizeof(cl_float));

    // Input and Output buffer
    cl_mem buffersIn[2] = {0, 0};
    cl_mem buffersOut[2] = {0, 0};

    // Bake the plan
    err = clfftBakePlan(planHandle, 1, &queue, NULL, NULL);
    // Size of temp buffer
    size_t tmpBufferSize = 0;
    int status = 0;
    // Create temporary buffer
    status = clfftGetTmpBufSize(planHandle, &tmpBufferSize);
    // Temporary buffer
    cl_mem tmpBuffer = 0;
    if ((status == 0) && (tmpBufferSize > 0))
    {
        tmpBuffer = clCreateBuffer(ctx, CL_MEM_READ_WRITE, tmpBufferSize, 0, &err);
        if (err != CL_SUCCESS)
        {
            qDebug() << "Error with tmpBuffer clCreateBuffer";
        }
    }
    float outRealCPU[length] = {0};
    float outImagCPU[length] = {0};

    for (int j = 0; j < length; j++)
    {
        inReal[j] = static_cast<float>(input[j].real());
        inImag[j] = static_cast<float>(input[j].imag());
        outReal[j] = 0.0;
        outImag[j] = 0.0;
    }
    // Prepare OpenCL memory objects : create buffer for input
    buffersIn[0] = clCreateBuffer(ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, length * sizeof(cl_float), inReal, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[0] clCreateBuffer";
    }

    err = clEnqueueWriteBuffer(queue, buffersIn[0], CL_TRUE, 0, length * sizeof(float), inReal, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[0] clEnqueueWriteBuffer";
    }
    // Prepare OpenCL memory objects : create buffer for input
    buffersIn[1] = clCreateBuffer(ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, length * sizeof(cl_float), inImag, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[1] clCreateBuffer";
    }

    // Enqueue write tab array into buffersIn[1]
    err = clEnqueueWriteBuffer(queue, buffersIn[1], CL_TRUE, 0, length * sizeof(float), inImag, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[1] clEnqueueWriteBuffer\n";
    }

    // Prepare OpenCL memory objects : create buffer for output
    buffersOut[0] = clCreateBuffer(ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, length * sizeof(cl_float), outReal, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersOut[1] clCreateBuffer";
    }

    buffersOut[1] = clCreateBuffer(ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, length * sizeof(cl_float), outImag, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersOut[1] clCreateBuffer";
    }

    // Execute the plan
    err = clfftEnqueueTransform(planHandle, CLFFT_FORWARD, 1, &queue, 0, NULL, NULL,
                                buffersIn, buffersOut, tmpBuffer);
    // Wait for calculations to be finished
    err = clFinish(queue);
    // Fetch results of calculations : Real and Imaginary
    err = clEnqueueReadBuffer(queue, buffersOut[0], CL_TRUE, 0, length * sizeof(float), &outRealCPU[0],
                              0, NULL, NULL);
    err = clEnqueueReadBuffer(queue, buffersOut[1], CL_TRUE, 0, length * sizeof(float), &outImagCPU[0],
                              0, NULL, NULL);

    for (int k = 0; k < length; k++)
    {
        retVal[k] = std::complex<double>(outRealCPU[k], outImagCPU[k]);
    }

    // Release OpenCL memory objects
    clReleaseMemObject(buffersIn[0]);
    clReleaseMemObject(buffersIn[1]);
    clReleaseMemObject(buffersOut[0]);
    clReleaseMemObject(buffersOut[1]);
    clReleaseMemObject(tmpBuffer);

    delete inReal;
    delete inImag;
    delete outReal;
    delete outImag;

    // Release the plan
    err = clfftDestroyPlan(&planHandle);

    // Release clFFT library
    clfftTeardown();

    // Release OpenCL working objects
    clReleaseCommandQueue(queue);
    clReleaseContext(ctx);
    return retVal;
}

std::complex<double> *GPSOpenCl::Utils::ifftComplex(std::complex<double> input[], int length)
{
    std::complex<double> *retVal = (std::complex<double> *)malloc(sizeof(std::complex<double>) * length);
    // OpenCL variables
    cl_int err;
    cl_platform_id platform = 0;
    cl_device_id device = 0;
    cl_context ctx = 0;
    cl_command_queue queue = 0;

    // Setup OpenCL environment
    err = clGetPlatformIDs(1, &platform, NULL);
    err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, NULL);

    // Create an OpenCL context
    ctx = clCreateContext(NULL, 1, &device, NULL, NULL, &err);

    // Create a command queue
    queue = clCreateCommandQueueWithProperties(ctx, device, 0, &err);

    // Setup clFFT
    clfftSetupData fftSetup;
    err = clfftInitSetupData(&fftSetup);
    err = clfftSetup(&fftSetup);

    // FFT library realted declarations
    clfftPlanHandle planHandle;
    clfftDim dim = CLFFT_1D;
    size_t clLengths[1] = {static_cast<size_t>(length)};

    // Create a default plan for a complex FFT
    err = clfftCreateDefaultPlan(&planHandle, ctx, dim, clLengths);

    // Set plan parameters
    err = clfftSetPlanPrecision(planHandle, CLFFT_SINGLE);
    err = clfftSetLayout(planHandle, CLFFT_COMPLEX_PLANAR, CLFFT_COMPLEX_PLANAR);
    err = clfftSetResultLocation(planHandle, CLFFT_OUTOFPLACE);

    // Real and Imaginary arrays
    cl_float *inReal = (cl_float *)malloc(length * sizeof(cl_float));
    cl_float *inImag = (cl_float *)malloc(length * sizeof(cl_float));
    cl_float *outReal = (cl_float *)malloc(length * sizeof(cl_float));
    cl_float *outImag = (cl_float *)malloc(length * sizeof(cl_float));

    // Input and Output buffer
    cl_mem buffersIn[2] = {0, 0};
    cl_mem buffersOut[2] = {0, 0};

    // Bake the plan
    err = clfftBakePlan(planHandle, 1, &queue, NULL, NULL);
    // Size of temp buffer
    size_t tmpBufferSize = 0;
    int status = 0;
    // Create temporary buffer
    status = clfftGetTmpBufSize(planHandle, &tmpBufferSize);
    // Temporary buffer
    cl_mem tmpBuffer = 0;
    if ((status == 0) && (tmpBufferSize > 0))
    {
        tmpBuffer = clCreateBuffer(ctx, CL_MEM_READ_WRITE, tmpBufferSize, 0, &err);
        if (err != CL_SUCCESS)
        {
            qDebug() << "Error with tmpBuffer clCreateBuffer";
        }
    }
    float outRealCPU[length] = {0};
    float outImagCPU[length] = {0};

    for (int j = 0; j < length; j++)
    {
        inReal[j] = static_cast<float>(input[j].real());
        inImag[j] = static_cast<float>(input[j].imag());
        outReal[j] = 0.0;
        outImag[j] = 0.0;
    }
    // Prepare OpenCL memory objects : create buffer for input
    buffersIn[0] = clCreateBuffer(ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, length * sizeof(cl_float), inReal, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[0] clCreateBuffer";
    }

    err = clEnqueueWriteBuffer(queue, buffersIn[0], CL_TRUE, 0, length * sizeof(float), inReal, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[0] clEnqueueWriteBuffer";
    }
    // Prepare OpenCL memory objects : create buffer for input
    buffersIn[1] = clCreateBuffer(ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, length * sizeof(cl_float), inImag, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[1] clCreateBuffer";
    }

    // Enqueue write tab array into buffersIn[1]
    err = clEnqueueWriteBuffer(queue, buffersIn[1], CL_TRUE, 0, length * sizeof(float), inImag, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersIn[1] clEnqueueWriteBuffer\n";
    }

    // Prepare OpenCL memory objects : create buffer for output
    buffersOut[0] = clCreateBuffer(ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, length * sizeof(cl_float), outReal, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersOut[1] clCreateBuffer";
    }

    buffersOut[1] = clCreateBuffer(ctx, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, length * sizeof(cl_float), outImag, &err);
    if (err != CL_SUCCESS)
    {
        qDebug() << "Error with buffersOut[1] clCreateBuffer";
    }

    // Execute the plan
    err = clfftEnqueueTransform(planHandle, CLFFT_BACKWARD, 1, &queue, 0, NULL, NULL,
                                buffersIn, buffersOut, tmpBuffer);
    // Wait for calculations to be finished
    err = clFinish(queue);
    // Fetch results of calculations : Real and Imaginary
    err = clEnqueueReadBuffer(queue, buffersOut[0], CL_TRUE, 0, length * sizeof(float), &outRealCPU[0],
                              0, NULL, NULL);
    err = clEnqueueReadBuffer(queue, buffersOut[1], CL_TRUE, 0, length * sizeof(float), &outImagCPU[0],
                              0, NULL, NULL);

    for (int k = 0; k < length; k++)
    {
        retVal[k] = std::complex<double>(outRealCPU[k], outImagCPU[k]);
    }

    // Release OpenCL memory objects
    clReleaseMemObject(buffersIn[0]);
    clReleaseMemObject(buffersIn[1]);
    clReleaseMemObject(buffersOut[0]);
    clReleaseMemObject(buffersOut[1]);
    clReleaseMemObject(tmpBuffer);

    delete inReal;
    delete inImag;
    delete outReal;
    delete outImag;

    // Release the plan
    err = clfftDestroyPlan(&planHandle);

    // Release clFFT library
    clfftTeardown();

    // Release OpenCL working objects
    clReleaseCommandQueue(queue);
    clReleaseContext(ctx);
    return retVal;
}

std::complex<double> *GPSOpenCl::Utils::conj(std::complex<double> input[], int length)
{
    std::complex<double> *retVal = (std::complex<double> *)malloc(sizeof(std::complex<double>) * length);
    for (int i = 0; i < length; i++)
    {
        retVal[i] = std::conj(input[i]);
    }
    return retVal;
}

std::complex<double> *GPSOpenCl::Utils::exp(int length, double frequency, double samplingRate)
{
    std::complex<double> *retVal = (std::complex<double> *)malloc(sizeof(std::complex<double>) * length);
    const std::complex<double> i(0, 1);
    const double pi = std::acos(-1);
    for (int sample = 0; sample < length; sample++)
    {
        double sampDouble = static_cast<double>(sample);
        retVal[sample] = std::exp(i * 2.0 * pi * frequency * sampDouble * (1 / samplingRate));
    }
    return retVal;
}

double* GPSOpenCl::Utils::abs(std::complex<double>* input, int length)
{
    double* retVal = (double *) malloc(sizeof(double) * length); 
    for (int i = 0; i < length; i++)
    {
        retVal[i] = std::abs(input[i]);
    }
    return retVal;
}