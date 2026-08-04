#include "GPSOpenClGPUCompute.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

using namespace GPSOpenCl;

unsigned int Compute::roundDownToPowerOfTwo(unsigned int value)
{
    if (value == 0)
    {
        return 0;
    }
    return static_cast<unsigned int>(pow(2, trunc(log2(static_cast<double>(value)))));
}

unsigned int Compute::clampPointsPerGroup(unsigned int pointsPerGroup, unsigned int length)
{
    pointsPerGroup = std::max(roundDownToPowerOfTwo(pointsPerGroup), 1u);
    return std::min(pointsPerGroup, length);
}

unsigned int Compute::clampLocalSizeForMinPointsPerItem(unsigned int localSize,
                                                        unsigned int pointsPerGroup,
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
        m_queue = clCreateCommandQueueWithProperties(m_gpu.m_context, m_gpu.m_device, nullptr, &m_error);
        if (m_error < 0)
        {
            std::cout << "Couldn't create a command queue" << '\n';
            m_gpu.getLastErrorAsString();
        }
    }
}

Compute::~Compute()
{
    auto releaseIfSet = [](cl_mem &mem)
    {
        if (mem != nullptr)
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

    if (m_queue != nullptr)
    {
        clReleaseCommandQueue(m_queue);
        m_queue = nullptr;
    }
}

void Compute::cacheDeviceInfo()
{
    if (m_deviceInfoCached || (m_queue == nullptr))
    {
        return;
    }

    m_gpu.determineLocalMemorySize();
    m_localMemorySize = m_gpu.m_localMemorySize;

    auto queryLocalSize = [&](cl_kernel kernel) -> size_t
    {
        size_t localSize = 0;
        const cl_int err = clGetKernelWorkGroupInfo(
            kernel, m_gpu.m_device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(localSize), &localSize, nullptr);
        if (err != CL_SUCCESS || localSize == 0)
        {
            return 0;
        }
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

    m_deviceInfoCached = true;
}

cl_mem Compute::ensureBuffer(cl_mem &buffer, size_t &capacityFloats, size_t neededFloats)
{
    if ((buffer != nullptr) && capacityFloats >= neededFloats)
    {
        m_error = CL_SUCCESS;
        return buffer;
    }

    if (buffer != nullptr)
    {
        clReleaseMemObject(buffer);
        buffer = nullptr;
        capacityFloats = 0;
    }

    buffer = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE, neededFloats * sizeof(float), nullptr, &m_error);
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

int Compute::fftDeviceInPlace(cl_mem buffer, unsigned int length, FFTDirectionType direction)
{
    if ((m_queue == nullptr) || m_gpu.m_acquisitionKernelList.size() <= GpuHandler::FFTScale)
    {
        return -1;
    }

    cacheDeviceInfo();

    cl_kernel initKernel = m_gpu.m_acquisitionKernelList[GpuHandler::FFTInit];
    cl_kernel stageKernel = m_gpu.m_acquisitionKernelList[GpuHandler::FFTStage];
    cl_kernel scaleKernel = m_gpu.m_acquisitionKernelList[GpuHandler::FFTScale];

    int dir = static_cast<int>(direction);
    size_t localSize = m_fftLocalSize;
    const cl_ulong localMemorySize = m_localMemorySize;
    unsigned int pointsPerGroup = localMemorySize / (2 * sizeof(float));

    pointsPerGroup = clampPointsPerGroup(pointsPerGroup, length);

    localSize = clampLocalSizeForMinPointsPerItem(static_cast<unsigned int>(localSize), pointsPerGroup, 4);

    m_error = clSetKernelArg(initKernel, 0, sizeof(cl_mem), reinterpret_cast<const void *>(&buffer));
    m_error |= clSetKernelArg(initKernel, 1, localMemorySize, nullptr);
    m_error |= clSetKernelArg(initKernel, 2, sizeof(pointsPerGroup), &pointsPerGroup);
    m_error |= clSetKernelArg(initKernel, 3, sizeof(length), &length);
    m_error |= clSetKernelArg(initKernel, 4, sizeof(dir), &dir);

    if (m_error != CL_SUCCESS)
    {
        return -1;
    }

    const size_t globalSize = (length / pointsPerGroup) * localSize;
    m_error = clEnqueueNDRangeKernel(m_queue, initKernel, 1, nullptr, &globalSize, &localSize, 0, nullptr, nullptr);

    if (m_error == CL_SUCCESS && length > pointsPerGroup)
    {
        m_error = clSetKernelArg(stageKernel, 0, sizeof(cl_mem), reinterpret_cast<const void *>(&buffer));
        m_error |= clSetKernelArg(stageKernel, 2, sizeof(pointsPerGroup), &pointsPerGroup);
        m_error |= clSetKernelArg(stageKernel, 3, sizeof(dir), &dir);

        for (unsigned int stage = 2; m_error == CL_SUCCESS && stage <= length / pointsPerGroup; stage <<= 1)
        {
            clSetKernelArg(stageKernel, 1, sizeof(stage), &stage);
            m_error =
                clEnqueueNDRangeKernel(m_queue, stageKernel, 1, nullptr, &globalSize, &localSize, 0, nullptr, nullptr);
        }
    }

    if (m_error == CL_SUCCESS && dir < 0)
    {
        m_error = clSetKernelArg(scaleKernel, 0, sizeof(cl_mem), reinterpret_cast<const void *>(&buffer));
        m_error |= clSetKernelArg(scaleKernel, 1, sizeof(pointsPerGroup), &pointsPerGroup);
        m_error |= clSetKernelArg(scaleKernel, 2, sizeof(length), &length);

        if (m_error == CL_SUCCESS)
        {
            m_error =
                clEnqueueNDRangeKernel(m_queue, scaleKernel, 1, nullptr, &globalSize, &localSize, 0, nullptr, nullptr);
        }
    }

    return (m_error == CL_SUCCESS) ? 0 : -1;
}

int Compute::fft(const ComplexFloatVector &input, ComplexFloatVector *output, FFTDirectionType direction)
{
    if ((m_queue != nullptr) && m_gpu.m_acquisitionKernelList.size() > GpuHandler::FFTScale)
    {
        const unsigned int length = input.size();

        cacheDeviceInfo();

        if (m_allocatedMemory.size() < 2UL * length)
        {
            m_allocatedMemory.resize(2UL * length);
        }

        for (unsigned int j = 0; j < length; j++)
        {
            const float realVal = std::real(input.at(j));
            const float imagVal = std::imag(input.at(j));

            m_allocatedMemory[2UL * j] = realVal;
            m_allocatedMemory[(2UL * j) + 1] = imagVal;
        }

        cl_mem dataBuffer = ensureBuffer(m_fftBuffer, m_fftBufferCapacity, 2UL * length);

        if ((dataBuffer != nullptr) && m_error == CL_SUCCESS)
        {
            m_error = clEnqueueWriteBuffer(m_queue,
                                           dataBuffer,
                                           CL_FALSE,
                                           0,
                                           2UL * length * sizeof(float),
                                           m_allocatedMemory.data(),
                                           0,
                                           nullptr,
                                           nullptr);
            if (m_error == CL_SUCCESS && fftDeviceInPlace(dataBuffer, length, direction) == 0)
            {
                m_error = clEnqueueReadBuffer(m_queue,
                                              dataBuffer,
                                              CL_TRUE,
                                              0,
                                              2UL * length * sizeof(float),
                                              m_allocatedMemory.data(),
                                              0,
                                              nullptr,
                                              nullptr);
                if (m_error == CL_SUCCESS)
                {
                    output->clear();
                    output->reserve(length);
                    for (unsigned int j = 0; j < length; j++)
                    {
                        const float realVal = m_allocatedMemory[2UL * j];
                        const float imagVal = m_allocatedMemory[(2UL * j) + 1];

                        output->emplace_back(realVal, imagVal);
                    }

                    return 0;
                }
            }
        }
    }

    output->clear();
    const size_t N = input.size();
    if (N == 0)
    {
        return 0;
    }

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
        while ((revIndex & bit) != 0u)
        {
            revIndex ^= bit;
            bit >>= 1;
        }
        revIndex ^= bit;
    }

    const float dirSign = (direction == FFTInverse) ? 1.0f : -1.0f;
    for (size_t len = 2; len <= N; len <<= 1)
    {
        const float ang = 2.0f * static_cast<float>(M_PI) / len * dirSign;
        const std::complex<float> wlen(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < N; i += len)
        {
            std::complex<float> w(1.0f, 0.0f);
            for (size_t k = 0; k < len / 2; k++)
            {
                const std::complex<float> u = output->at(i + k);
                const std::complex<float> v = output->at(i + k + (len / 2)) * w;
                output->at(i + k) = u + v;
                output->at(i + k + (len / 2)) = u - v;
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

cl_mem Compute::complexMultiplierDevice(const ComplexFloatVector &input1,
                                        const ComplexFloatVector &input2,
                                        unsigned int length)
{
    cacheDeviceInfo();

    if (m_allocatedMemory.size() < 4UL * length)
    {
        m_allocatedMemory.resize(4UL * length);
    }

    for (unsigned int j = 0; j < length; j++)
    {
        const float realVal = std::real(input1.at(j));
        const float imagVal = std::imag(input1.at(j));

        m_allocatedMemory[2UL * j] = realVal;
        m_allocatedMemory[(2UL * j) + 1] = imagVal;
    }

    for (unsigned int j = 0; j < length; j++)
    {
        const float realVal = std::real(input2.at(j));
        const float imagVal = std::imag(input2.at(j));

        m_allocatedMemory[(2UL * length) + (2UL * j)] = realVal;
        m_allocatedMemory[(2UL * length) + (2UL * j) + 1] = imagVal;
    }

    cl_kernel complexMultiplierKernel = m_gpu.m_acquisitionKernelList[GpuHandler::ComplexMultiplier];
    cl_mem dA = ensureBuffer(m_cmBufferA, m_cmBufferCapacityA, 2UL * length);
    cl_mem dB = ((dA != nullptr) && m_error == CL_SUCCESS)
        ? ensureBuffer(m_cmBufferB, m_cmBufferCapacityB, 2UL * length)
        : nullptr;
    cl_mem dC = ((dB != nullptr) && m_error == CL_SUCCESS)
        ? ensureBuffer(m_cmBufferC, m_cmBufferCapacityC, 2UL * length)
        : nullptr;

    if ((dA == nullptr) || (dB == nullptr) || (dC == nullptr) || m_error != CL_SUCCESS)
    {
        return nullptr;
    }

    clEnqueueWriteBuffer(
        m_queue, dA, CL_FALSE, 0, 2UL * length * sizeof(float), m_allocatedMemory.data(), 0, nullptr, nullptr);
    clEnqueueWriteBuffer(
        m_queue, dB, CL_FALSE, 0, 2UL * length * sizeof(float), &m_allocatedMemory[2UL * length], 0, nullptr, nullptr);

    size_t localSize = m_complexMultiplierLocalSize;
    const cl_ulong localMemorySize = m_localMemorySize;
    unsigned int pointsPerGroup = localMemorySize / (2 * sizeof(float));

    pointsPerGroup = clampPointsPerGroup(pointsPerGroup, length);
    localSize = clampLocalSizeForMinPointsPerItem(static_cast<unsigned int>(localSize), pointsPerGroup, 1);

    clSetKernelArg(complexMultiplierKernel, 0, sizeof(cl_mem), reinterpret_cast<const void *>(&dA));
    clSetKernelArg(complexMultiplierKernel, 1, sizeof(cl_mem), reinterpret_cast<const void *>(&dB));
    clSetKernelArg(complexMultiplierKernel, 2, sizeof(cl_mem), reinterpret_cast<const void *>(&dC));
    clSetKernelArg(complexMultiplierKernel, 3, sizeof(unsigned int), &pointsPerGroup);

    const size_t globalSize = (length / pointsPerGroup) * localSize;
    m_error = clEnqueueNDRangeKernel(
        m_queue, complexMultiplierKernel, 1, nullptr, &globalSize, &localSize, 0, nullptr, nullptr);

    return (m_error == CL_SUCCESS) ? dC : nullptr;
}

int Compute::complexMultiplier(const ComplexFloatVector &input1,
                               const ComplexFloatVector &input2,
                               ComplexFloatVector *output)
{
    if ((m_queue != nullptr) && m_gpu.m_acquisitionKernelList.size() > GpuHandler::ComplexMultiplier)
    {
        auto length = static_cast<unsigned int>(input1.size());
        cl_mem dC = complexMultiplierDevice(input1, input2, length);

        if ((dC != nullptr) && m_error == CL_SUCCESS)
        {
            m_error = clEnqueueReadBuffer(
                m_queue, dC, CL_TRUE, 0, 2UL * length * sizeof(float), m_allocatedMemory.data(), 0, nullptr, nullptr);

            if (m_error == CL_SUCCESS)
            {
                output->clear();
                output->reserve(length);
                for (unsigned int j = 0; j < length; j++)
                {
                    const float realVal = m_allocatedMemory[2UL * j];
                    const float imagVal = m_allocatedMemory[(2UL * j) + 1];

                    output->emplace_back(realVal, imagVal);
                }

                return 0;
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

int Compute::absoluteDeviceToHost(cl_mem inputBuffer, unsigned int length, FloatVector *output)
{
    if ((m_queue == nullptr) || m_gpu.m_acquisitionKernelList.size() <= GpuHandler::Absolute)
    {
        return -1;
    }

    cacheDeviceInfo();

    cl_kernel absoluteKernel = m_gpu.m_acquisitionKernelList[GpuHandler::Absolute];
    cl_mem dC = ensureBuffer(m_absBufferC, m_absBufferCapacityC, length);

    if ((dC == nullptr) || m_error != CL_SUCCESS)
    {
        return -1;
    }

    size_t localSize = m_absoluteLocalSize;
    const cl_ulong localMemorySize = m_localMemorySize;
    unsigned int pointsPerGroup = localMemorySize / (2 * sizeof(float));

    pointsPerGroup = clampPointsPerGroup(pointsPerGroup, length);
    localSize = clampLocalSizeForMinPointsPerItem(static_cast<unsigned int>(localSize), pointsPerGroup, 1);

    clSetKernelArg(absoluteKernel, 0, sizeof(cl_mem), reinterpret_cast<const void *>(&inputBuffer));
    clSetKernelArg(absoluteKernel, 1, sizeof(cl_mem), reinterpret_cast<const void *>(&dC));
    clSetKernelArg(absoluteKernel, 2, sizeof(unsigned int), &pointsPerGroup);

    const size_t globalSize = (length / pointsPerGroup) * localSize;
    m_error = clEnqueueNDRangeKernel(m_queue, absoluteKernel, 1, nullptr, &globalSize, &localSize, 0, nullptr, nullptr);

    if (m_error == CL_SUCCESS)
    {
        m_error = clEnqueueReadBuffer(
            m_queue, dC, CL_TRUE, 0, length * sizeof(float), m_allocatedMemory.data(), 0, nullptr, nullptr);

        if (m_error == CL_SUCCESS)
        {
            output->assign(m_allocatedMemory.begin(), m_allocatedMemory.begin() + length);
            return 0;
        }
    }

    return -1;
}

int Compute::absolute(const ComplexFloatVector &input1, FloatVector *output)
{
    if ((m_queue != nullptr) && m_gpu.m_acquisitionKernelList.size() > GpuHandler::Absolute)
    {
        auto length = static_cast<unsigned int>(input1.size());

        cacheDeviceInfo();

        if (m_allocatedMemory.size() < 2UL * length)
        {
            m_allocatedMemory.resize(2UL * length);
        }

        for (unsigned int j = 0; j < length; j++)
        {
            const float realVal = std::real(input1.at(j));
            const float imagVal = std::imag(input1.at(j));

            m_allocatedMemory[2UL * j] = realVal;
            m_allocatedMemory[(2UL * j) + 1] = imagVal;
        }

        cl_mem dA = ensureBuffer(m_absBufferA, m_absBufferCapacityA, 2UL * length);

        if ((dA != nullptr) && m_error == CL_SUCCESS)
        {
            clEnqueueWriteBuffer(
                m_queue, dA, CL_FALSE, 0, 2UL * length * sizeof(float), m_allocatedMemory.data(), 0, nullptr, nullptr);

            if (absoluteDeviceToHost(dA, length, output) == 0)
            {
                return 0;
            }
        }
    }

    output->clear();
    output->reserve(input1.size());
    for (auto j : input1)
    {
        const float realVal = std::real(j);
        const float imagVal = std::imag(j);
        output->push_back((realVal * realVal) + (imagVal * imagVal));
    }
    return 0;
}

int Compute::complexMultiplyThenFft(const ComplexFloatVector &input1,
                                    const ComplexFloatVector &input2,
                                    FFTDirectionType direction,
                                    ComplexFloatVector *output)
{
    auto length = static_cast<unsigned int>(input1.size());
    if ((m_queue != nullptr) && m_gpu.m_acquisitionKernelList.size() > GpuHandler::FFTScale &&
        m_gpu.m_acquisitionKernelList.size() > GpuHandler::ComplexMultiplier)
    {
        cl_mem product = complexMultiplierDevice(input1, input2, length);
        if ((product != nullptr) && m_error == CL_SUCCESS && fftDeviceInPlace(product, length, direction) == 0)
        {
            m_error = clEnqueueReadBuffer(m_queue,
                                          product,
                                          CL_TRUE,
                                          0,
                                          2UL * length * sizeof(float),
                                          m_allocatedMemory.data(),
                                          0,
                                          nullptr,
                                          nullptr);
            if (m_error == CL_SUCCESS)
            {
                output->clear();
                output->reserve(length);
                for (unsigned int j = 0; j < length; j++)
                {
                    output->emplace_back(m_allocatedMemory[2UL * j], m_allocatedMemory[(2UL * j) + 1]);
                }
                return 0;
            }
        }
    }

    ComplexFloatVector product;
    complexMultiplier(input1, input2, &product);
    return fft(product, output, direction);
}

int Compute::complexMultiplyThenFftThenAbsolute(const ComplexFloatVector &input1,
                                                const ComplexFloatVector &input2,
                                                FFTDirectionType direction,
                                                FloatVector *output)
{
    auto length = static_cast<unsigned int>(input1.size());
    if ((m_queue != nullptr) && m_gpu.m_acquisitionKernelList.size() > GpuHandler::FFTScale &&
        m_gpu.m_acquisitionKernelList.size() > GpuHandler::ComplexMultiplier &&
        m_gpu.m_acquisitionKernelList.size() > GpuHandler::Absolute)
    {
        cl_mem product = complexMultiplierDevice(input1, input2, length);
        if ((product != nullptr) && m_error == CL_SUCCESS && fftDeviceInPlace(product, length, direction) == 0 &&
            absoluteDeviceToHost(product, length, output) == 0)
        {
            return 0;
        }
    }

    ComplexFloatVector product;
    complexMultiplier(input1, input2, &product);
    ComplexFloatVector fftResult;
    if (fft(product, &fftResult, direction) != 0)
    {
        return -1;
    }
    return absolute(fftResult, output);
}

int Compute::sum(const FloatVector &input, float *sumValue)
{
    if ((m_queue != nullptr) && m_gpu.m_acquisitionKernelList.size() > GpuHandler::Sum && !input.empty())
    {
        auto length = static_cast<unsigned int>(input.size());

        cl_kernel sumKernel = m_gpu.m_acquisitionKernelList[GpuHandler::Sum];

        cacheDeviceInfo();
        const size_t localSize = m_sumLocalSize;

        if (localSize > 0)
        {
            const size_t paddedLength = ((static_cast<size_t>(length) + localSize - 1) / localSize) * localSize;
            const size_t numGroups = paddedLength / localSize;

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

            cl_mem dInput = ensureBuffer(m_sumBufferInput, m_sumBufferInputCapacity, paddedLength);
            cl_mem dSumValue = ((dInput != nullptr) && m_error == CL_SUCCESS)
                ? ensureBuffer(m_sumBufferOutput, m_sumBufferOutputCapacity, numGroups)
                : nullptr;

            if ((dInput != nullptr) && (dSumValue != nullptr) && m_error == CL_SUCCESS)
            {
                // Single dispatch: one work-group per localSize-sized chunk, each group's partial
                // sum lands in its own slot of dSumValue, instead of a chunked host round-trip loop.
                m_error = clEnqueueWriteBuffer(m_queue,
                                               dInput,
                                               CL_FALSE,
                                               0,
                                               paddedLength * sizeof(float),
                                               m_allocatedMemory.data(),
                                               0,
                                               nullptr,
                                               nullptr);

                clSetKernelArg(sumKernel, 0, sizeof(cl_mem), reinterpret_cast<const void *>(&dInput));
                clSetKernelArg(sumKernel, 1, sizeof(cl_mem), reinterpret_cast<const void *>(&dSumValue));
                clSetKernelArg(sumKernel, 2, sizeof(float) * localSize, nullptr);

                const size_t globalSize = paddedLength;
                m_error = clEnqueueNDRangeKernel(
                    m_queue, sumKernel, 1, nullptr, &globalSize, &localSize, 0, nullptr, nullptr);

                if (m_error == CL_SUCCESS)
                {
                    if (m_partialSums.size() < numGroups)
                    {
                        m_partialSums.resize(numGroups);
                    }
                    m_error = clEnqueueReadBuffer(m_queue,
                                                  dSumValue,
                                                  CL_TRUE,
                                                  0,
                                                  numGroups * sizeof(float),
                                                  m_partialSums.data(),
                                                  0,
                                                  nullptr,
                                                  nullptr);
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
    for (const float val : input)
    {
        total += val;
    }
    *sumValue += total;
    return 0;
}
