#include "GPSOpenClGPUCompute.h"

#include <cmath>
#include <cstring>
#include <iostream>

using namespace GPSOpenCl;

unsigned int Compute::roundDownToPowerOfTwo(unsigned int value)
{
    if (value == 0) return 0;
    return static_cast<unsigned int>(pow(2, trunc(log2(static_cast<double>(value)))));
}

unsigned int Compute::clampLocalSizeForMinPointsPerItem(unsigned int localSize, unsigned int pointsPerGroup,
                                                        unsigned int minPointsPerItem)
{
    while (localSize > pointsPerGroup / minPointsPerItem && localSize > 1)
    {
        localSize >>= 1;
    }
    return localSize;
}





Compute::Compute() : m_queue(nullptr), m_error(-1)
{
    if (m_gpu.createDevice() >= 0)
    {
        if (m_gpu.buildProgram() >= 0)
        {
            m_gpu.initKernels();
        }
        m_queue = clCreateCommandQueueWithProperties(m_gpu.m_context, m_gpu.m_device, NULL, &m_error);
        if (m_error < 0)
        {
            std::cout << "Couldn't create a command queue" << std::endl;
            m_gpu.getLastErrorAsString();
        }
    }

    m_allocatedMemory.resize(DEFAULT_MAX_ALLOCATION * 4, 0.0f);
}





Compute::~Compute()
{
    if (m_queue)
    {
        clReleaseCommandQueue(m_queue);
        m_queue = nullptr;
    }
}









int Compute::fft(const ComplexFloatVector &input, ComplexFloatVector *output, FFTDirectionType direction)
{
    if (m_queue && m_gpu.m_acquisitionKernelList.size() > GpuHandler::FFTScale)
    {
        size_t global_size = 0;
        size_t local_size = 0;
        unsigned int points_per_group = 0;
        unsigned int stage = 0;
        unsigned int length = input.size();
        int dir = static_cast<int>(direction);
        cl_ulong localMemorySize;

        cl_kernel initKernel = m_gpu.m_acquisitionKernelList[GpuHandler::FFTInit];
        cl_kernel stageKernel = m_gpu.m_acquisitionKernelList[GpuHandler::FFTStage];
        cl_kernel scaleKernel = m_gpu.m_acquisitionKernelList[GpuHandler::FFTScale];

        if (m_allocatedMemory.size() < 2 * length)
        {
            m_allocatedMemory.resize(2 * length);
        }

        for (unsigned int j = 0; j < length; j++)
        {
            float realVal = std::real(input.at(j));
            float imagVal = std::imag(input.at(j));

            m_allocatedMemory[2 * j] = realVal;
            m_allocatedMemory[2 * j + 1] = imagVal;
        }

        cl_mem dataBuffer = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                                           2 * length * sizeof(float), m_allocatedMemory.data(), &m_error);

        if (m_error == CL_SUCCESS)
        {
            m_error = clGetKernelWorkGroupInfo(initKernel, m_gpu.m_device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(local_size),
                                               &local_size, NULL);
            if (m_error == CL_SUCCESS)
            {
                local_size = (int)pow(2, trunc(log2(local_size)));
                m_error = m_gpu.determineLocalMemorySize();
                if (CL_SUCCESS == m_error)
                {
                    localMemorySize = m_gpu.m_localMemorySize;
                    points_per_group = localMemorySize / (2 * sizeof(float));




                    points_per_group = roundDownToPowerOfTwo(points_per_group);
                    if (points_per_group > length)
                    {
                        points_per_group = length;
                    }






                    local_size = clampLocalSizeForMinPointsPerItem(static_cast<unsigned int>(local_size), points_per_group, 4);

                    m_error = clSetKernelArg(initKernel, 0, sizeof(cl_mem), &dataBuffer);
                    m_error |= clSetKernelArg(initKernel, 1, localMemorySize, NULL);
                    m_error |= clSetKernelArg(initKernel, 2, sizeof(points_per_group), &points_per_group);
                    m_error |= clSetKernelArg(initKernel, 3, sizeof(length), &length);
                    m_error |= clSetKernelArg(initKernel, 4, sizeof(dir), &dir);

                    if (m_error == CL_SUCCESS)
                    {
                        global_size = (length / points_per_group) * local_size;
                        m_error = clEnqueueNDRangeKernel(m_queue, initKernel, 1, NULL, &global_size, &local_size, 0, NULL, NULL);

                        if (m_error == CL_SUCCESS && length > points_per_group)
                        {
                            m_error = clSetKernelArg(stageKernel, 0, sizeof(cl_mem), &dataBuffer);
                            m_error |= clSetKernelArg(stageKernel, 2, sizeof(points_per_group), &points_per_group);
                            m_error |= clSetKernelArg(stageKernel, 3, sizeof(dir), &dir);

                            for (stage = 2; m_error == CL_SUCCESS && stage <= length / points_per_group; stage <<= 1)
                            {
                                clSetKernelArg(stageKernel, 1, sizeof(stage), &stage);
                                m_error = clEnqueueNDRangeKernel(m_queue, stageKernel, 1, NULL, &global_size, &local_size, 0, NULL, NULL);
                            }
                        }

                        if (m_error == CL_SUCCESS && dir < 0)
                        {
                            m_error = clSetKernelArg(scaleKernel, 0, sizeof(cl_mem), &dataBuffer);
                            m_error |= clSetKernelArg(scaleKernel, 1, sizeof(points_per_group), &points_per_group);
                            m_error |= clSetKernelArg(scaleKernel, 2, sizeof(length), &length);

                            if (m_error == CL_SUCCESS)
                            {
                                m_error = clEnqueueNDRangeKernel(m_queue, scaleKernel, 1, NULL, &global_size, &local_size, 0, NULL, NULL);
                            }
                        }

                        if (m_error == CL_SUCCESS)
                        {
                            m_error = clEnqueueReadBuffer(m_queue, dataBuffer, CL_TRUE, 0, 2 * length * sizeof(float),
                                                          m_allocatedMemory.data(), 0, NULL, NULL);
                            if (m_error == CL_SUCCESS)
                            {
                                output->clear();
                                output->reserve(length);
                                for (unsigned int j = 0; j < length; j++)
                                {
                                    float realVal = m_allocatedMemory[2 * j];
                                    float imagVal = m_allocatedMemory[2 * j + 1];

                                    output->push_back(std::complex<float>(realVal, imagVal));
                                }

                                clReleaseMemObject(dataBuffer);
                                return 0;
                            }
                        }
                    }
                }
            }
            clReleaseMemObject(dataBuffer);
        }
    }


    output->clear();
    size_t N = input.size();
    if (N == 0) return 0;

    auto isPowerOfTwo = [](size_t n) { return n > 0 && (n & (n - 1)) == 0; };
    if (!isPowerOfTwo(N))
    {
        return -1;
    }

    output->resize(N);

    size_t revIndex = 0;
    for (size_t i = 0; i < N; i++)
    {
        output->at(revIndex) = input[i];
        size_t bit = N >> 1;
        while (revIndex & bit)
        {
            revIndex ^= bit;
            bit >>= 1;
        }
        revIndex ^= bit;
    }


    float dirSign = (direction == FFTInverse) ? 1.0f : -1.0f;
    for (size_t len = 2; len <= N; len <<= 1)
    {
        float ang = 2.0f * static_cast<float>(M_PI) / len * dirSign;
        std::complex<float> wlen(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < N; i += len)
        {
            std::complex<float> w(1.0f, 0.0f);
            for (size_t k = 0; k < len / 2; k++)
            {
                std::complex<float> u = output->at(i + k);
                std::complex<float> v = output->at(i + k + len / 2) * w;
                output->at(i + k) = u + v;
                output->at(i + k + len / 2) = u - v;
                w *= wlen;
            }
        }
    }

    if (direction == FFTInverse)
    {
        for (size_t i = 0; i < N; i++)
        {
            output->at(i) /= static_cast<float>(N);
        }
    }

    return 0;
}

int Compute::complexMultiplier(const ComplexFloatVector &input1, const ComplexFloatVector &input2,
                               ComplexFloatVector *output)
{
    if (m_queue && m_gpu.m_acquisitionKernelList.size() > GpuHandler::ComplexMultiplier)
    {
        size_t global_size = 0;
        size_t local_size = 0;
        unsigned int length = static_cast<unsigned int>(input1.size());
        cl_mem d_a;
        cl_mem d_b;
        cl_mem d_c;
        cl_ulong localMemorySize;
        unsigned int points_per_group = 0;

        if (m_allocatedMemory.size() < 4 * length)
        {
            m_allocatedMemory.resize(4 * length);
        }

        for (unsigned int j = 0; j < length; j++)
        {
            float realVal = std::real(input1.at(j));
            float imagVal = std::imag(input1.at(j));

            m_allocatedMemory[2 * j] = realVal;
            m_allocatedMemory[2 * j + 1] = imagVal;
        }

        for (unsigned int j = 0; j < length; j++)
        {
            float realVal = std::real(input2.at(j));
            float imagVal = std::imag(input2.at(j));

            m_allocatedMemory[(2 * length) + 2 * j] = realVal;
            m_allocatedMemory[(2 * length) + 2 * j + 1] = imagVal;
        }

        cl_kernel complexMultiplierKernel = m_gpu.m_acquisitionKernelList[GpuHandler::ComplexMultiplier];
        d_a = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE, 2 * length * sizeof(float), NULL, &m_error);
        if (m_error != CL_SUCCESS)
        {
            goto complexMultiplierCpuFallback;
        }
        d_b = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE, 2 * length * sizeof(float), NULL, &m_error);
        if (m_error != CL_SUCCESS)
        {
            clReleaseMemObject(d_a);
            goto complexMultiplierCpuFallback;
        }
        d_c = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE, 2 * length * sizeof(float), NULL, &m_error);
        if (m_error != CL_SUCCESS)
        {
            clReleaseMemObject(d_a);
            clReleaseMemObject(d_b);
            goto complexMultiplierCpuFallback;
        }

        {
            clEnqueueWriteBuffer(m_queue, d_a, CL_TRUE, 0, 2 * length * sizeof(float), m_allocatedMemory.data(), 0, NULL, NULL);
            clEnqueueWriteBuffer(m_queue, d_b, CL_TRUE, 0, 2 * length * sizeof(float), &m_allocatedMemory[2 * length], 0, NULL, NULL);

            m_error = clGetKernelWorkGroupInfo(complexMultiplierKernel, m_gpu.m_device, CL_KERNEL_WORK_GROUP_SIZE,
                                               sizeof(local_size), &local_size, NULL);
            if (m_error == CL_SUCCESS)
            {
                local_size = (int)pow(2, trunc(log2(local_size)));
                m_error = m_gpu.determineLocalMemorySize();
                if (m_error == CL_SUCCESS)
                {
                    localMemorySize = m_gpu.m_localMemorySize;
                    points_per_group = localMemorySize / (2 * sizeof(float));


                    points_per_group = roundDownToPowerOfTwo(points_per_group);
                    if (points_per_group > length)
                    {
                        points_per_group = length;
                    }
                    local_size = clampLocalSizeForMinPointsPerItem(static_cast<unsigned int>(local_size), points_per_group, 1);

                    clSetKernelArg(complexMultiplierKernel, 0, sizeof(cl_mem), &d_a);
                    clSetKernelArg(complexMultiplierKernel, 1, sizeof(cl_mem), &d_b);
                    clSetKernelArg(complexMultiplierKernel, 2, sizeof(cl_mem), &d_c);
                    clSetKernelArg(complexMultiplierKernel, 3, sizeof(unsigned int), &points_per_group);

                    global_size = (length / points_per_group) * local_size;
                    m_error = clEnqueueNDRangeKernel(m_queue, complexMultiplierKernel, 1, NULL, &global_size, &local_size, 0, NULL, NULL);

                    if (m_error == CL_SUCCESS)
                    {
                        clEnqueueReadBuffer(m_queue, d_c, CL_TRUE, 0, 2 * length * sizeof(float), m_allocatedMemory.data(), 0, NULL, NULL);

                        output->clear();
                        output->reserve(length);
                        for (unsigned int j = 0; j < length; j++)
                        {
                            float realVal = m_allocatedMemory[2 * j];
                            float imagVal = m_allocatedMemory[2 * j + 1];

                            output->push_back(std::complex<float>(realVal, imagVal));
                        }

                        clReleaseMemObject(d_a);
                        clReleaseMemObject(d_b);
                        clReleaseMemObject(d_c);
                        return 0;
                    }
                }
            }
            clReleaseMemObject(d_a);
            clReleaseMemObject(d_b);
            clReleaseMemObject(d_c);
        }
        complexMultiplierCpuFallback:;
    }


    output->clear();
    output->reserve(input1.size());
    for (size_t j = 0; j < input1.size(); j++)
    {
        output->push_back(input1[j] * input2[j]);
    }
    return 0;
}

int Compute::absolute(const ComplexFloatVector &input1, FloatVector *output)
{
    if (m_queue && m_gpu.m_acquisitionKernelList.size() > GpuHandler::Absolute)
    {
        size_t global_size = 0;
        size_t local_size = 0;
        unsigned int length = static_cast<unsigned int>(input1.size());
        cl_mem d_a;
        cl_mem d_c;
        cl_ulong localMemorySize;
        unsigned int points_per_group = 0;

        if (m_allocatedMemory.size() < 2 * length)
        {
            m_allocatedMemory.resize(2 * length);
        }

        for (unsigned int j = 0; j < length; j++)
        {
            float realVal = std::real(input1.at(j));
            float imagVal = std::imag(input1.at(j));

            m_allocatedMemory[2 * j] = realVal;
            m_allocatedMemory[2 * j + 1] = imagVal;
        }

        cl_kernel absoluteKernel = m_gpu.m_acquisitionKernelList[GpuHandler::Absolute];
        d_a = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE, 2 * length * sizeof(float), NULL, &m_error);
        if (m_error != CL_SUCCESS)
        {
            goto absoluteCpuFallback;
        }
        d_c = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE, 2 * length * sizeof(float), NULL, &m_error);
        if (m_error != CL_SUCCESS)
        {
            clReleaseMemObject(d_a);
            goto absoluteCpuFallback;
        }

        {
            clEnqueueWriteBuffer(m_queue, d_a, CL_TRUE, 0, 2 * length * sizeof(float), m_allocatedMemory.data(), 0, NULL, NULL);

            m_error = clGetKernelWorkGroupInfo(absoluteKernel, m_gpu.m_device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(local_size),
                                               &local_size, NULL);
            if (m_error == CL_SUCCESS)
            {
                local_size = (int)pow(2, trunc(log2(local_size)));
                m_error = m_gpu.determineLocalMemorySize();
                if (m_error == CL_SUCCESS)
                {
                    localMemorySize = m_gpu.m_localMemorySize;
                    points_per_group = localMemorySize / (2 * sizeof(float));


                    points_per_group = roundDownToPowerOfTwo(points_per_group);
                    if (points_per_group > length)
                    {
                        points_per_group = length;
                    }
                    local_size = clampLocalSizeForMinPointsPerItem(static_cast<unsigned int>(local_size), points_per_group, 1);

                    clSetKernelArg(absoluteKernel, 0, sizeof(cl_mem), &d_a);
                    clSetKernelArg(absoluteKernel, 1, sizeof(cl_mem), &d_c);
                    clSetKernelArg(absoluteKernel, 2, sizeof(unsigned int), &points_per_group);

                    global_size = (length / points_per_group) * local_size;
                    m_error = clEnqueueNDRangeKernel(m_queue, absoluteKernel, 1, NULL, &global_size, &local_size, 0, NULL, NULL);

                    if (m_error == CL_SUCCESS)
                    {
                        clEnqueueReadBuffer(m_queue, d_c, CL_TRUE, 0, 2 * length * sizeof(float), m_allocatedMemory.data(), 0, NULL, NULL);

                        output->clear();
                        output->reserve(length);
                        for (unsigned int j = 0; j < length; j++)
                        {
                            float value = m_allocatedMemory[2 * j];
                            output->push_back(value);
                        }

                        clReleaseMemObject(d_a);
                        clReleaseMemObject(d_c);
                        return 0;
                    }
                }
            }
            clReleaseMemObject(d_a);
            clReleaseMemObject(d_c);
        }
        absoluteCpuFallback:;
    }


    output->clear();
    output->reserve(input1.size());
    for (size_t j = 0; j < input1.size(); j++)
    {
        float realVal = std::real(input1[j]);
        float imagVal = std::imag(input1[j]);
        output->push_back(realVal * realVal + imagVal * imagVal);
    }
    return 0;
}

int Compute::sum(const FloatVector &input, float *sumValue)
{
    if (m_queue && m_gpu.m_acquisitionKernelList.size() > GpuHandler::Sum && !input.empty())
    {
        size_t global_size = 0;
        size_t local_size = 0;
        cl_mem d_input;
        cl_mem d_sumValue;
        cl_ulong localMemorySize;
        unsigned int length = static_cast<unsigned int>(input.size());
        unsigned int points_per_group = 0;
        float tmpSumValue = 0.0f;

        cl_kernel sumKernel = m_gpu.m_acquisitionKernelList[GpuHandler::Sum];

        m_error = clGetKernelWorkGroupInfo(sumKernel, m_gpu.m_device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(local_size),
                                           &local_size, NULL);
        if (m_error == CL_SUCCESS)
        {
            local_size = (int)pow(2, trunc(log2(local_size)));
            m_error = m_gpu.determineLocalMemorySize();
            if (m_error == CL_SUCCESS)
            {
                localMemorySize = m_gpu.m_localMemorySize;
                points_per_group = localMemorySize / sizeof(float);
                if (points_per_group > length)
                {
                    points_per_group = length;
                }



                size_t paddedLength = ((static_cast<size_t>(length) + local_size - 1) / local_size) * local_size;
                if (m_allocatedMemory.size() < paddedLength)
                {
                    m_allocatedMemory.resize(paddedLength);
                }

                for (unsigned int j = 0; j < length; j++)
                {
                    m_allocatedMemory[j] = input.at(j);
                }
                for (size_t j = length; j < paddedLength; j++)
                {
                    m_allocatedMemory[j] = 0.0f;
                }

                d_input = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE, local_size * sizeof(float), NULL, &m_error);
                d_sumValue = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE, sizeof(float), NULL, &m_error);

                if (m_error == CL_SUCCESS)
                {
                    global_size = local_size;
                    size_t computedSize = 0;
                    do
                    {
                        m_error = clEnqueueWriteBuffer(m_queue, d_input, CL_TRUE, 0, local_size * sizeof(float),
                                                       &m_allocatedMemory[computedSize], 0, NULL, NULL);

                        clSetKernelArg(sumKernel, 0, sizeof(cl_mem), &d_input);
                        clSetKernelArg(sumKernel, 1, sizeof(cl_mem), &d_sumValue);
                        clSetKernelArg(sumKernel, 2, sizeof(float) * local_size, NULL);

                        m_error = clEnqueueNDRangeKernel(m_queue, sumKernel, 1, NULL, &global_size, &local_size, 0, NULL, NULL);
                        m_error = clEnqueueReadBuffer(m_queue, d_sumValue, CL_TRUE, 0, sizeof(float), &tmpSumValue, 0, NULL, NULL);

                        computedSize += local_size;
                        *sumValue += tmpSumValue;
                    } while (m_error == CL_SUCCESS && computedSize < paddedLength);

                    clReleaseMemObject(d_input);
                    clReleaseMemObject(d_sumValue);
                    if (m_error == CL_SUCCESS)
                    {
                        return 0;
                    }
                }
            }
        }
    }


    float total = 0.0f;
    for (float val : input)
    {
        total += val;
    }
    *sumValue += total;
    return 0;
}

int Compute::ncoMultiplication(const ComplexFloatVector &input, const FloatVector &phaseVector,
                               ComplexFloatVector *output)
{
    if (m_queue && m_gpu.m_trackingKernelList.size() > GpuHandler::NCOMultiplicate)
    {
        size_t global_size = 0;
        size_t local_size = 0;
        unsigned int points_per_group = 0;
        unsigned int length = input.size();
        cl_ulong localMemorySize;

        cl_kernel ncoMultiplicationKernel = m_gpu.m_trackingKernelList[GpuHandler::NCOMultiplicate];

        if (m_allocatedMemory.size() < 3 * length)
        {
            m_allocatedMemory.resize(3 * length);
        }

        for (unsigned int j = 0; j < length; j++)
        {
            float realVal = std::real(input.at(j));
            float imagVal = std::imag(input.at(j));

            m_allocatedMemory[2 * j] = realVal;
            m_allocatedMemory[2 * j + 1] = imagVal;
        }

        for (unsigned int j = 0; j < length; j++)
        {
            m_allocatedMemory[2 * length + j] = phaseVector.at(j);
        }

        cl_mem dataBuffer = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                                           2 * length * sizeof(float), m_allocatedMemory.data(), &m_error);

        cl_mem phaseBuffer = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                                            length * sizeof(float), &m_allocatedMemory[2 * length], &m_error);

        if (m_error == CL_SUCCESS)
        {
            m_error = clGetKernelWorkGroupInfo(ncoMultiplicationKernel, m_gpu.m_device, CL_KERNEL_WORK_GROUP_SIZE,
                                               sizeof(local_size), &local_size, NULL);
            if (m_error == CL_SUCCESS)
            {
                local_size = (int)pow(2, trunc(log2(local_size)));
                m_error = m_gpu.determineLocalMemorySize();
                if (m_error == CL_SUCCESS)
                {
                    localMemorySize = m_gpu.m_localMemorySize;
                    points_per_group = localMemorySize / (2 * sizeof(float));


                    points_per_group = roundDownToPowerOfTwo(points_per_group);
                    points_per_group = (points_per_group > length) ? length : points_per_group;




                    local_size = clampLocalSizeForMinPointsPerItem(static_cast<unsigned int>(local_size), points_per_group, 4);

                    clSetKernelArg(ncoMultiplicationKernel, 0, sizeof(cl_mem), &dataBuffer);
                    clSetKernelArg(ncoMultiplicationKernel, 1, sizeof(cl_mem), &phaseBuffer);
                    clSetKernelArg(ncoMultiplicationKernel, 2, localMemorySize, NULL);
                    clSetKernelArg(ncoMultiplicationKernel, 3, sizeof(unsigned int), &points_per_group);

                    global_size = (length / points_per_group) * local_size;
                    m_error = clEnqueueNDRangeKernel(m_queue, ncoMultiplicationKernel, 1, NULL, &global_size, &local_size, 0, NULL, NULL);

                    if (m_error == CL_SUCCESS)
                    {
                        m_error = clEnqueueReadBuffer(m_queue, dataBuffer, CL_TRUE, 0, 2 * length * sizeof(float), m_allocatedMemory.data(), 0,
                                                      NULL, NULL);
                        if (m_error == CL_SUCCESS)
                        {
                            output->clear();
                            output->reserve(length);
                            for (unsigned int j = 0; j < length; j++)
                            {
                                float realVal = m_allocatedMemory[2 * j];
                                float imagVal = m_allocatedMemory[2 * j + 1];

                                output->push_back(std::complex<float>(realVal, imagVal));
                            }

                            clReleaseMemObject(dataBuffer);
                            clReleaseMemObject(phaseBuffer);
                            return 0;
                        }
                    }
                }
            }
            clReleaseMemObject(dataBuffer);
            clReleaseMemObject(phaseBuffer);
        }
    }


    output->clear();
    output->reserve(input.size());
    for (size_t j = 0; j < input.size(); j++)
    {
        float p = phaseVector[j];
        std::complex<float> nco(std::cos(p), std::sin(p));
        output->push_back(input[j] * nco);
    }
    return 0;
}
