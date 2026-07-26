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
}





Compute::~Compute()
{
    auto releaseIfSet = [](cl_mem &mem) {
        if (mem)
        {
            clReleaseMemObject(mem);
            mem = nullptr;
        }
    };

    releaseIfSet(m_fftBuffer);
    releaseIfSet(m_cmBufferA);
    releaseIfSet(m_cmBufferB);
    releaseIfSet(m_cmBufferC);
    releaseIfSet(m_absBufferA);
    releaseIfSet(m_absBufferC);
    releaseIfSet(m_sumBufferInput);
    releaseIfSet(m_sumBufferOutput);
    releaseIfSet(m_ncoBufferData);
    releaseIfSet(m_ncoBufferPhase);

    if (m_queue)
    {
        clReleaseCommandQueue(m_queue);
        m_queue = nullptr;
    }
}

void Compute::cacheDeviceInfo()
{
    if (m_deviceInfoCached || !m_queue) return;

    m_gpu.determineLocalMemorySize();
    m_localMemorySize = m_gpu.m_localMemorySize;

    auto queryLocalSize = [&](cl_kernel kernel) -> size_t {
        size_t localSize = 0;
        cl_int err =
            clGetKernelWorkGroupInfo(kernel, m_gpu.m_device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(localSize), &localSize, NULL);
        if (err != CL_SUCCESS || localSize == 0) return 0;
        return static_cast<size_t>(pow(2, trunc(log2(static_cast<double>(localSize)))));
    };

    if (m_gpu.m_acquisitionKernelList.size() > GpuHandler::FFTInit)
    {
        m_fftLocalSize = queryLocalSize(m_gpu.m_acquisitionKernelList[GpuHandler::FFTInit]);
    }
    if (m_gpu.m_acquisitionKernelList.size() > GpuHandler::ComplexMultiplier)
    {
        m_complexMultiplierLocalSize = queryLocalSize(m_gpu.m_acquisitionKernelList[GpuHandler::ComplexMultiplier]);
    }
    if (m_gpu.m_acquisitionKernelList.size() > GpuHandler::Absolute)
    {
        m_absoluteLocalSize = queryLocalSize(m_gpu.m_acquisitionKernelList[GpuHandler::Absolute]);
    }
    if (m_gpu.m_acquisitionKernelList.size() > GpuHandler::Sum)
    {
        m_sumLocalSize = queryLocalSize(m_gpu.m_acquisitionKernelList[GpuHandler::Sum]);
    }
    if (m_gpu.m_trackingKernelList.size() > GpuHandler::NCOMultiplicate)
    {
        m_ncoLocalSize = queryLocalSize(m_gpu.m_trackingKernelList[GpuHandler::NCOMultiplicate]);
    }

    m_deviceInfoCached = true;
}

cl_mem Compute::ensureBuffer(cl_mem &buffer, size_t &capacityFloats, size_t neededFloats)
{
    if (buffer && capacityFloats >= neededFloats)
    {
        m_error = CL_SUCCESS;
        return buffer;
    }

    if (buffer)
    {
        clReleaseMemObject(buffer);
        buffer = nullptr;
        capacityFloats = 0;
    }

    buffer = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE, neededFloats * sizeof(float), NULL, &m_error);
    if (m_error == CL_SUCCESS)
    {
        capacityFloats = neededFloats;
    }
    else
    {
        buffer = nullptr;
        capacityFloats = 0;
    }
    return buffer;
}









int Compute::fft(const ComplexFloatVector &input, ComplexFloatVector *output, FFTDirectionType direction)
{
    if (m_queue && m_gpu.m_acquisitionKernelList.size() > GpuHandler::FFTScale)
    {
        size_t global_size = 0;
        unsigned int points_per_group = 0;
        unsigned int stage = 0;
        unsigned int length = input.size();
        int dir = static_cast<int>(direction);

        cl_kernel initKernel = m_gpu.m_acquisitionKernelList[GpuHandler::FFTInit];
        cl_kernel stageKernel = m_gpu.m_acquisitionKernelList[GpuHandler::FFTStage];
        cl_kernel scaleKernel = m_gpu.m_acquisitionKernelList[GpuHandler::FFTScale];

        cacheDeviceInfo();

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

        cl_mem dataBuffer = ensureBuffer(m_fftBuffer, m_fftBufferCapacity, 2 * length);

        if (dataBuffer && m_error == CL_SUCCESS)
        {
            m_error = clEnqueueWriteBuffer(m_queue, dataBuffer, CL_FALSE, 0, 2 * length * sizeof(float),
                                           m_allocatedMemory.data(), 0, NULL, NULL);
            if (m_error == CL_SUCCESS)
            {
                size_t local_size = m_fftLocalSize;
                cl_ulong localMemorySize = m_localMemorySize;
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

                            return 0;
                        }
                    }
                }
            }
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
        unsigned int length = static_cast<unsigned int>(input1.size());
        unsigned int points_per_group = 0;

        cacheDeviceInfo();

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
        cl_mem d_a = ensureBuffer(m_cmBufferA, m_cmBufferCapacityA, 2 * length);
        cl_mem d_b = (d_a && m_error == CL_SUCCESS) ? ensureBuffer(m_cmBufferB, m_cmBufferCapacityB, 2 * length) : nullptr;
        cl_mem d_c = (d_b && m_error == CL_SUCCESS) ? ensureBuffer(m_cmBufferC, m_cmBufferCapacityC, 2 * length) : nullptr;

        if (d_a && d_b && d_c && m_error == CL_SUCCESS)
        {
            clEnqueueWriteBuffer(m_queue, d_a, CL_FALSE, 0, 2 * length * sizeof(float), m_allocatedMemory.data(), 0, NULL, NULL);
            clEnqueueWriteBuffer(m_queue, d_b, CL_FALSE, 0, 2 * length * sizeof(float), &m_allocatedMemory[2 * length], 0, NULL, NULL);

            size_t local_size = m_complexMultiplierLocalSize;
            cl_ulong localMemorySize = m_localMemorySize;
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
                m_error = clEnqueueReadBuffer(m_queue, d_c, CL_TRUE, 0, 2 * length * sizeof(float), m_allocatedMemory.data(), 0, NULL, NULL);

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

                    return 0;
                }
            }
        }
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
        unsigned int length = static_cast<unsigned int>(input1.size());
        unsigned int points_per_group = 0;

        cacheDeviceInfo();

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
        cl_mem d_a = ensureBuffer(m_absBufferA, m_absBufferCapacityA, 2 * length);
        cl_mem d_c = (d_a && m_error == CL_SUCCESS) ? ensureBuffer(m_absBufferC, m_absBufferCapacityC, 2 * length) : nullptr;

        if (d_a && d_c && m_error == CL_SUCCESS)
        {
            clEnqueueWriteBuffer(m_queue, d_a, CL_FALSE, 0, 2 * length * sizeof(float), m_allocatedMemory.data(), 0, NULL, NULL);

            size_t local_size = m_absoluteLocalSize;
            cl_ulong localMemorySize = m_localMemorySize;
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
                m_error = clEnqueueReadBuffer(m_queue, d_c, CL_TRUE, 0, 2 * length * sizeof(float), m_allocatedMemory.data(), 0, NULL, NULL);

                if (m_error == CL_SUCCESS)
                {
                    output->clear();
                    output->reserve(length);
                    for (unsigned int j = 0; j < length; j++)
                    {
                        float value = m_allocatedMemory[2 * j];
                        output->push_back(value);
                    }

                    return 0;
                }
            }
        }
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
        unsigned int length = static_cast<unsigned int>(input.size());

        cl_kernel sumKernel = m_gpu.m_acquisitionKernelList[GpuHandler::Sum];

        cacheDeviceInfo();
        size_t local_size = m_sumLocalSize;

        if (local_size > 0)
        {
            size_t paddedLength = ((static_cast<size_t>(length) + local_size - 1) / local_size) * local_size;
            size_t numGroups = paddedLength / local_size;

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

            cl_mem d_input = ensureBuffer(m_sumBufferInput, m_sumBufferInputCapacity, paddedLength);
            cl_mem d_sumValue =
                (d_input && m_error == CL_SUCCESS) ? ensureBuffer(m_sumBufferOutput, m_sumBufferOutputCapacity, numGroups) : nullptr;

            if (d_input && d_sumValue && m_error == CL_SUCCESS)
            {
                // Single dispatch: one work-group per local_size-sized chunk, each group's partial
                // sum lands in its own slot of d_sumValue, instead of a chunked host round-trip loop.
                m_error = clEnqueueWriteBuffer(m_queue, d_input, CL_FALSE, 0, paddedLength * sizeof(float),
                                               m_allocatedMemory.data(), 0, NULL, NULL);

                clSetKernelArg(sumKernel, 0, sizeof(cl_mem), &d_input);
                clSetKernelArg(sumKernel, 1, sizeof(cl_mem), &d_sumValue);
                clSetKernelArg(sumKernel, 2, sizeof(float) * local_size, NULL);

                size_t global_size = paddedLength;
                m_error = clEnqueueNDRangeKernel(m_queue, sumKernel, 1, NULL, &global_size, &local_size, 0, NULL, NULL);

                if (m_error == CL_SUCCESS)
                {
                    if (m_partialSums.size() < numGroups)
                    {
                        m_partialSums.resize(numGroups);
                    }
                    m_error = clEnqueueReadBuffer(m_queue, d_sumValue, CL_TRUE, 0, numGroups * sizeof(float),
                                                  m_partialSums.data(), 0, NULL, NULL);
                }

                if (m_error == CL_SUCCESS)
                {
                    float total = 0.0f;
                    for (size_t g = 0; g < numGroups; g++)
                    {
                        total += m_partialSums[g];
                    }
                    *sumValue += total;
                    return 0;
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
        unsigned int points_per_group = 0;
        unsigned int length = input.size();

        cl_kernel ncoMultiplicationKernel = m_gpu.m_trackingKernelList[GpuHandler::NCOMultiplicate];

        cacheDeviceInfo();

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

        cl_mem dataBuffer = ensureBuffer(m_ncoBufferData, m_ncoBufferDataCapacity, 2 * length);
        cl_mem phaseBuffer =
            (dataBuffer && m_error == CL_SUCCESS) ? ensureBuffer(m_ncoBufferPhase, m_ncoBufferPhaseCapacity, length) : nullptr;

        if (dataBuffer && phaseBuffer && m_error == CL_SUCCESS)
        {
            m_error = clEnqueueWriteBuffer(m_queue, dataBuffer, CL_FALSE, 0, 2 * length * sizeof(float),
                                           m_allocatedMemory.data(), 0, NULL, NULL);
            m_error |= clEnqueueWriteBuffer(m_queue, phaseBuffer, CL_FALSE, 0, length * sizeof(float),
                                            &m_allocatedMemory[2 * length], 0, NULL, NULL);

            if (m_error == CL_SUCCESS)
            {
                size_t local_size = m_ncoLocalSize;
                cl_ulong localMemorySize = m_localMemorySize;
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

                        return 0;
                    }
                }
            }
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
