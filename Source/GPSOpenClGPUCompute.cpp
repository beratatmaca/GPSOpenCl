#include "GPSOpenClGPUCompute.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
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

namespace
{
bool deviceSupportsOpenCl2(cl_device_id device)
{
    char versionString[64] = {0};
    const cl_int err = clGetDeviceInfo(device, CL_DEVICE_VERSION, sizeof(versionString) - 1, versionString, nullptr);
    if (err != CL_SUCCESS)
    {
        return false;
    }
    int major = 0;
    return (sscanf(versionString, "OpenCL %d", &major) == 1) && major >= 2;
}
}

Compute::Compute()
{
    if (m_gpu.createDevice() >= 0)
    {
        if (m_gpu.buildProgram() >= 0)
        {
            m_gpu.initKernels();
        }
        if (deviceSupportsOpenCl2(m_gpu.device))
        {
            m_queue = clCreateCommandQueueWithProperties(m_gpu.context, m_gpu.device, nullptr, &m_error);
        }
        else
        {
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
            m_queue = clCreateCommandQueue(m_gpu.context, m_gpu.device, 0, &m_error);
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
        }
        if (m_error < 0)
        {
            std::cout << "Couldn't create a command queue" << '\n';
            m_gpu.getLastErrorAsString();
        }
    }
}

Compute::~Compute()
{
    if (m_queue != nullptr)
    {
        clFinish(m_queue);
    }

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
    releaseIfSet(m_slotPoolBuffer);
    releaseIfSet(m_batchWorkBuffer);
    releaseIfSet(m_batchAbsBuffer);
    releaseIfSet(m_batchParamsBuffer);

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
    m_localMemorySize = m_gpu.localMemorySize;

    auto queryLocalSize = [&](cl_kernel kernel) -> size_t
    {
        size_t localSize = 0;
        const cl_int err = clGetKernelWorkGroupInfo(
            kernel, m_gpu.device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(localSize), &localSize, nullptr);
        if (err != CL_SUCCESS || localSize == 0)
        {
            return 0;
        }
        return static_cast<size_t>(pow(2, trunc(log2(static_cast<double>(localSize)))));
    };

    if (m_gpu.acquisitionKernelList.size() > GpuHandler::FFTInit)
    {
        m_fftLocalSize = queryLocalSize(m_gpu.acquisitionKernelList[GpuHandler::FFTInit]);
    }
    if (m_gpu.acquisitionKernelList.size() > GpuHandler::ComplexMultiplier)
    {
        m_complexMultiplierLocalSize = queryLocalSize(m_gpu.acquisitionKernelList[GpuHandler::ComplexMultiplier]);
    }
    if (m_gpu.acquisitionKernelList.size() > GpuHandler::Absolute)
    {
        m_absoluteLocalSize = queryLocalSize(m_gpu.acquisitionKernelList[GpuHandler::Absolute]);
    }
    if (m_gpu.acquisitionKernelList.size() > GpuHandler::Sum)
    {
        m_sumLocalSize = queryLocalSize(m_gpu.acquisitionKernelList[GpuHandler::Sum]);
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

    buffer = clCreateBuffer(m_gpu.context, CL_MEM_READ_WRITE, neededFloats * sizeof(float), nullptr, &m_error);
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
    if ((m_queue == nullptr) || m_gpu.acquisitionKernelList.size() <= GpuHandler::FFTScale)
    {
        return -1;
    }
    if (length == 0 || (length & (length - 1)) != 0)
    {
        return -1;
    }

    cacheDeviceInfo();

    cl_kernel initKernel = m_gpu.acquisitionKernelList[GpuHandler::FFTInit];
    cl_kernel stageKernel = m_gpu.acquisitionKernelList[GpuHandler::FFTStage];
    cl_kernel scaleKernel = m_gpu.acquisitionKernelList[GpuHandler::FFTScale];

    int dir = static_cast<int>(direction);
    size_t localSize = m_fftLocalSize;
    const cl_ulong localMemorySize = m_localMemorySize;
    unsigned int pointsPerGroup = localMemorySize / (2 * sizeof(float));

    pointsPerGroup = clampPointsPerGroup(pointsPerGroup, length);

    localSize = clampLocalSizeForMinPointsPerItem(static_cast<unsigned int>(localSize), pointsPerGroup, 4);

    m_error = clSetKernelArg(initKernel, 0, sizeof(cl_mem), reinterpret_cast<const void *>(&buffer));
    m_error |= clSetKernelArg(initKernel, 1, pointsPerGroup * 2UL * sizeof(float), nullptr);
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

int Compute::fftDeviceInPlaceBatch(cl_mem buffer, unsigned int length, unsigned int count, FFTDirectionType direction)
{
    if ((m_queue == nullptr) || m_gpu.acquisitionKernelList.size() <= GpuHandler::FFTScale)
    {
        return -1;
    }
    if (length == 0 || (length & (length - 1)) != 0)
    {
        return -1;
    }

    cacheDeviceInfo();

    cl_kernel initKernel = m_gpu.acquisitionKernelList[GpuHandler::FFTInit];
    cl_kernel scaleKernel = m_gpu.acquisitionKernelList[GpuHandler::FFTScale];

    int dir = static_cast<int>(direction);
    size_t localSize = m_fftLocalSize;
    const cl_ulong localMemorySize = m_localMemorySize;
    unsigned int pointsPerGroup = localMemorySize / (2 * sizeof(float));

    pointsPerGroup = clampPointsPerGroup(pointsPerGroup, length);
    if (pointsPerGroup != length || localSize == 0)
    {
        return -1;
    }
    localSize = clampLocalSizeForMinPointsPerItem(static_cast<unsigned int>(localSize), pointsPerGroup, 4);

    m_error = clSetKernelArg(initKernel, 0, sizeof(cl_mem), reinterpret_cast<const void *>(&buffer));
    m_error |= clSetKernelArg(initKernel, 1, pointsPerGroup * 2UL * sizeof(float), nullptr);
    m_error |= clSetKernelArg(initKernel, 2, sizeof(pointsPerGroup), &pointsPerGroup);
    m_error |= clSetKernelArg(initKernel, 3, sizeof(length), &length);
    m_error |= clSetKernelArg(initKernel, 4, sizeof(dir), &dir);

    if (m_error != CL_SUCCESS)
    {
        return -1;
    }

    const size_t globalSize = static_cast<size_t>(count) * localSize;
    m_error = clEnqueueNDRangeKernel(m_queue, initKernel, 1, nullptr, &globalSize, &localSize, 0, nullptr, nullptr);

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
    if ((m_queue != nullptr) && m_gpu.acquisitionKernelList.size() > GpuHandler::FFTScale)
    {
        const unsigned int length = input.size();

        cacheDeviceInfo();

        packToStaging(input, length, 0);

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
                    output->resize(length);
                    std::memcpy(output->data(), m_allocatedMemory.data(), 2UL * length * sizeof(float));

                    return 0;
                }
            }
        }
    }

    if (m_queue != nullptr)
    {
        clFinish(m_queue);
    }

    return fftCpu(input, output, direction);
}

int Compute::fftCpu(const ComplexFloatVector &input, ComplexFloatVector *output, FFTDirectionType direction)
{
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
    std::complex<float> *data = output->data();

    size_t revIndex = 0;
    for (size_t i = 0; i < N; i++)
    {
        data[revIndex] = input[i];
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
        const float ang = 2.0f * static_cast<float>(M_PI) / static_cast<float>(len) * dirSign;
        const std::complex<float> wlen(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < N; i += len)
        {
            std::complex<float> w(1.0f, 0.0f);
            for (size_t k = 0; k < len / 2; k++)
            {
                const std::complex<float> u = data[i + k];
                const std::complex<float> v = data[i + k + (len / 2)] * w;
                data[i + k] = u + v;
                data[i + k + (len / 2)] = u - v;
                w *= wlen;
            }
        }
    }

    if (direction == FFTInverse)
    {
        for (size_t i = 0; i < N; i++)
        {
            data[i] /= static_cast<float>(N);
        }
    }

    return 0;
}

void Compute::packToStaging(const ComplexFloatVector &input, unsigned int length, size_t floatOffset)
{
    if (m_allocatedMemory.size() < floatOffset + (2UL * length))
    {
        m_allocatedMemory.resize(floatOffset + (2UL * length));
    }

    std::memcpy(&m_allocatedMemory[floatOffset], input.data(), 2UL * length * sizeof(float));
}

cl_mem Compute::enqueueComplexMultiplier(cl_mem inputA,
                                         cl_mem inputB,
                                         unsigned int length,
                                         unsigned int offset,
                                         unsigned int inputBBase)
{
    cl_kernel complexMultiplierKernel = m_gpu.acquisitionKernelList[GpuHandler::ComplexMultiplier];
    cl_mem dC = ensureBuffer(m_cmBufferC, m_cmBufferCapacityC, 2UL * length);

    if ((dC == nullptr) || m_error != CL_SUCCESS)
    {
        return nullptr;
    }

    size_t localSize = m_complexMultiplierLocalSize;
    unsigned int pointsPerGroup = clampPointsPerGroup(static_cast<unsigned int>(localSize), length);
    localSize = clampLocalSizeForMinPointsPerItem(static_cast<unsigned int>(localSize), pointsPerGroup, 1);

    m_error = clSetKernelArg(complexMultiplierKernel, 0, sizeof(cl_mem), reinterpret_cast<const void *>(&inputA));
    m_error |= clSetKernelArg(complexMultiplierKernel, 1, sizeof(cl_mem), reinterpret_cast<const void *>(&inputB));
    m_error |= clSetKernelArg(complexMultiplierKernel, 2, sizeof(cl_mem), reinterpret_cast<const void *>(&dC));
    m_error |= clSetKernelArg(complexMultiplierKernel, 3, sizeof(unsigned int), &pointsPerGroup);
    m_error |= clSetKernelArg(complexMultiplierKernel, 4, sizeof(unsigned int), &length);
    m_error |= clSetKernelArg(complexMultiplierKernel, 5, sizeof(unsigned int), &offset);
    m_error |= clSetKernelArg(complexMultiplierKernel, 6, sizeof(unsigned int), &inputBBase);

    if (m_error != CL_SUCCESS)
    {
        return nullptr;
    }

    const size_t globalSize = (length / pointsPerGroup) * localSize;
    m_error = clEnqueueNDRangeKernel(
        m_queue, complexMultiplierKernel, 1, nullptr, &globalSize, &localSize, 0, nullptr, nullptr);

    return (m_error == CL_SUCCESS) ? dC : nullptr;
}

cl_mem Compute::complexMultiplierDevice(const ComplexFloatVector &input1,
                                        const ComplexFloatVector &input2,
                                        unsigned int length)
{
    cacheDeviceInfo();

    packToStaging(input1, length, 0);
    packToStaging(input2, length, 2UL * length);

    cl_mem dA = ensureBuffer(m_cmBufferA, m_cmBufferCapacityA, 2UL * length);
    cl_mem dB = ((dA != nullptr) && m_error == CL_SUCCESS)
        ? ensureBuffer(m_cmBufferB, m_cmBufferCapacityB, 2UL * length)
        : nullptr;

    if ((dA == nullptr) || (dB == nullptr) || m_error != CL_SUCCESS)
    {
        return nullptr;
    }

    m_cachedInput1Ptr = nullptr;
    m_cachedInput1Len = 0;

    m_error = clEnqueueWriteBuffer(
        m_queue, dA, CL_FALSE, 0, 2UL * length * sizeof(float), m_allocatedMemory.data(), 0, nullptr, nullptr);
    if (m_error == CL_SUCCESS)
    {
        m_error = clEnqueueWriteBuffer(m_queue,
                                       dB,
                                       CL_FALSE,
                                       0,
                                       2UL * length * sizeof(float),
                                       &m_allocatedMemory[2UL * length],
                                       0,
                                       nullptr,
                                       nullptr);
    }
    if (m_error != CL_SUCCESS)
    {
        clFinish(m_queue);
        return nullptr;
    }

    return enqueueComplexMultiplier(dA, dB, length, 0, 0);
}

cl_mem Compute::ensureResidentInput1(const ComplexFloatVector &input1, unsigned int length)
{
    const size_t capacityBefore = m_cmBufferCapacityA;
    cl_mem dA = ensureBuffer(m_cmBufferA, m_cmBufferCapacityA, 2UL * length);
    if ((dA == nullptr) || m_error != CL_SUCCESS)
    {
        return nullptr;
    }

    const bool recreated = (m_cmBufferCapacityA != capacityBefore);
    if (recreated || (m_cachedInput1Ptr != input1.data()) || (m_cachedInput1Len != length))
    {
        packToStaging(input1, length, 0);
        m_error = clEnqueueWriteBuffer(
            m_queue, dA, CL_TRUE, 0, 2UL * length * sizeof(float), m_allocatedMemory.data(), 0, nullptr, nullptr);
        if (m_error != CL_SUCCESS)
        {
            m_cachedInput1Ptr = nullptr;
            m_cachedInput1Len = 0;
            return nullptr;
        }
        m_cachedInput1Ptr = input1.data();
        m_cachedInput1Len = length;
    }

    return dA;
}

cl_mem Compute::complexMultiplierResidentDevice(const ComplexFloatVector &input1,
                                                cl_mem input2,
                                                unsigned int length,
                                                unsigned int offset)
{
    cacheDeviceInfo();

    cl_mem dA = ensureResidentInput1(input1, length);
    if (dA == nullptr)
    {
        return nullptr;
    }

    return enqueueComplexMultiplier(dA, input2, length, offset, 0);
}

cl_mem Compute::ensureSlotPool(int slot, size_t floatsPerSlot)
{
    const size_t minimumSlots = 32;
    const size_t neededSlots = ((static_cast<size_t>(slot) / minimumSlots) + 1) * minimumSlots;

    if (m_slotPoolStrideFloats != floatsPerSlot)
    {
        std::fill(m_slotStates.begin(), m_slotStates.end(), SlotInvalid);
        m_slotPoolStrideFloats = floatsPerSlot;
    }

    const size_t capacityBefore = m_slotPoolCapacity;
    cl_mem pool = ensureBuffer(m_slotPoolBuffer, m_slotPoolCapacity, neededSlots * floatsPerSlot);
    if (pool != nullptr && m_slotPoolCapacity != capacityBefore)
    {
        std::fill(m_slotStates.begin(), m_slotStates.end(), SlotInvalid);
    }
    return pool;
}

int Compute::complexMultiplier(const ComplexFloatVector &input1,
                               const ComplexFloatVector &input2,
                               ComplexFloatVector *output)
{
    if ((m_queue != nullptr) && m_gpu.acquisitionKernelList.size() > GpuHandler::ComplexMultiplier)
    {
        auto length = static_cast<unsigned int>(input1.size());
        cl_mem dC = complexMultiplierDevice(input1, input2, length);

        if ((dC != nullptr) && m_error == CL_SUCCESS)
        {
            m_error = clEnqueueReadBuffer(
                m_queue, dC, CL_TRUE, 0, 2UL * length * sizeof(float), m_allocatedMemory.data(), 0, nullptr, nullptr);

            if (m_error == CL_SUCCESS)
            {
                output->resize(length);
                std::memcpy(output->data(), m_allocatedMemory.data(), 2UL * length * sizeof(float));

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
    if ((m_queue == nullptr) || m_gpu.acquisitionKernelList.size() <= GpuHandler::Absolute)
    {
        return -1;
    }

    cacheDeviceInfo();

    cl_kernel absoluteKernel = m_gpu.acquisitionKernelList[GpuHandler::Absolute];
    cl_mem dC = ensureBuffer(m_absBufferC, m_absBufferCapacityC, length);

    if ((dC == nullptr) || m_error != CL_SUCCESS)
    {
        return -1;
    }

    size_t localSize = m_absoluteLocalSize;
    unsigned int pointsPerGroup = clampPointsPerGroup(static_cast<unsigned int>(localSize), length);
    localSize = clampLocalSizeForMinPointsPerItem(static_cast<unsigned int>(localSize), pointsPerGroup, 1);

    clSetKernelArg(absoluteKernel, 0, sizeof(cl_mem), reinterpret_cast<const void *>(&inputBuffer));
    clSetKernelArg(absoluteKernel, 1, sizeof(cl_mem), reinterpret_cast<const void *>(&dC));
    clSetKernelArg(absoluteKernel, 2, sizeof(unsigned int), &pointsPerGroup);

    const size_t globalSize = (length / pointsPerGroup) * localSize;
    m_error = clEnqueueNDRangeKernel(m_queue, absoluteKernel, 1, nullptr, &globalSize, &localSize, 0, nullptr, nullptr);

    if (m_error == CL_SUCCESS)
    {
        if (m_allocatedMemory.size() < length)
        {
            m_allocatedMemory.resize(length);
        }
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
    if ((m_queue != nullptr) && m_gpu.acquisitionKernelList.size() > GpuHandler::Absolute)
    {
        auto length = static_cast<unsigned int>(input1.size());

        cacheDeviceInfo();

        packToStaging(input1, length, 0);

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

int Compute::sum(const FloatVector &input, float *sumValue)
{
    if ((m_queue != nullptr) && m_gpu.acquisitionKernelList.size() > GpuHandler::Sum && !input.empty())
    {
        auto length = static_cast<unsigned int>(input.size());

        cl_kernel sumKernel = m_gpu.acquisitionKernelList[GpuHandler::Sum];

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

int Compute::complexMultiplyThenFftToSlot(const ComplexFloatVector &input1,
                                          const ComplexFloatVector &input2,
                                          FFTDirectionType direction,
                                          int slot)
{
    if (slot < 0)
    {
        return -1;
    }

    const auto slotIndex = static_cast<size_t>(slot);
    if (m_slotStates.size() <= slotIndex)
    {
        m_slotStates.resize(slotIndex + 1, SlotInvalid);
        m_slotLengths.resize(slotIndex + 1, 0);
        m_slotHost.resize(slotIndex + 1);
    }
    m_slotStates[slotIndex] = SlotInvalid;

    auto length = static_cast<unsigned int>(input1.size());
    if ((m_queue != nullptr) && m_gpu.acquisitionKernelList.size() > GpuHandler::FFTScale &&
        m_gpu.acquisitionKernelList.size() > GpuHandler::ComplexMultiplier)
    {
        cacheDeviceInfo();

        cl_mem dA = ensureResidentInput1(input1, length);
        cl_mem dB = (dA != nullptr) ? ensureBuffer(m_cmBufferB, m_cmBufferCapacityB, 2UL * length) : nullptr;
        if ((dA != nullptr) && (dB != nullptr) && m_error == CL_SUCCESS)
        {
            packToStaging(input2, length, 0);
            m_error = clEnqueueWriteBuffer(
                m_queue, dB, CL_TRUE, 0, 2UL * length * sizeof(float), m_allocatedMemory.data(), 0, nullptr, nullptr);

            cl_mem product = (m_error == CL_SUCCESS) ? enqueueComplexMultiplier(dA, dB, length, 0, 0) : nullptr;
            if ((product != nullptr) && m_error == CL_SUCCESS && fftDeviceInPlace(product, length, direction) == 0)
            {
                cl_mem slotPool = ensureSlotPool(slot, 2UL * length);
                if (slotPool != nullptr && m_error == CL_SUCCESS)
                {
                    const size_t destinationOffset = slotIndex * m_slotPoolStrideFloats * sizeof(float);
                    m_error = clEnqueueCopyBuffer(m_queue,
                                                  product,
                                                  slotPool,
                                                  0,
                                                  destinationOffset,
                                                  2UL * length * sizeof(float),
                                                  0,
                                                  nullptr,
                                                  nullptr);
                    if (m_error == CL_SUCCESS)
                    {
                        m_slotStates[slotIndex] = SlotDevice;
                        m_slotLengths[slotIndex] = length;
                        return 0;
                    }
                }
            }
        }
        if (m_queue != nullptr)
        {
            clFinish(m_queue);
        }
    }

    ComplexFloatVector product;
    complexMultiplier(input1, input2, &product);
    if (fft(product, &m_slotHost[slotIndex], direction) != 0)
    {
        return -1;
    }
    m_slotStates[slotIndex] = SlotHost;
    m_slotLengths[slotIndex] = length;
    return 0;
}

int Compute::complexMultiplyResidentThenFftThenAbsolute(const ComplexFloatVector &input1,
                                                        int slot,
                                                        int shiftBins,
                                                        FFTDirectionType direction,
                                                        FloatVector *output)
{
    const auto slotIndex = static_cast<size_t>(slot);
    if (slot < 0 || m_slotStates.size() <= slotIndex || m_slotStates[slotIndex] == SlotInvalid)
    {
        return -1;
    }

    const unsigned int length = m_slotLengths[slotIndex];
    if (length == 0 || input1.size() != length)
    {
        return -1;
    }

    const auto shift = static_cast<unsigned int>(((shiftBins % static_cast<int>(length)) + static_cast<int>(length)) %
                                                 static_cast<int>(length));
    const unsigned int offset = (length - shift) % length;

    if (m_slotStates[slotIndex] == SlotDevice)
    {
        cacheDeviceInfo();

        cl_mem dA = ensureResidentInput1(input1, length);
        if (dA == nullptr)
        {
            return -1;
        }

        const auto slotBase = static_cast<unsigned int>(slotIndex * (m_slotPoolStrideFloats / 2));
        cl_mem product = enqueueComplexMultiplier(dA, m_slotPoolBuffer, length, offset, slotBase);
        if ((product != nullptr) && m_error == CL_SUCCESS && fftDeviceInPlace(product, length, direction) == 0 &&
            absoluteDeviceToHost(product, length, output) == 0)
        {
            return 0;
        }
        return -1;
    }

    const ComplexFloatVector &resident = m_slotHost[slotIndex];
    m_hostProductScratch.resize(length);
    for (unsigned int i = 0; i < length; i++)
    {
        unsigned int j = i + offset;
        if (j >= length)
        {
            j -= length;
        }
        m_hostProductScratch[i] = input1[i] * resident[j];
    }

    if (fft(m_hostProductScratch, &m_hostFftScratch, direction) != 0)
    {
        return -1;
    }
    return absolute(m_hostFftScratch, output);
}

int Compute::complexMultiplyResidentThenFftThenAbsoluteBatch(const ComplexFloatVector &input1,
                                                             const std::vector<std::pair<int, int>> &slotAndShift,
                                                             FFTDirectionType direction,
                                                             FloatVector *output)
{
    const auto bins = static_cast<unsigned int>(slotAndShift.size());
    const auto length = static_cast<unsigned int>(input1.size());
    if (bins == 0 || length == 0)
    {
        return -1;
    }

    if ((m_queue == nullptr) || m_gpu.acquisitionKernelList.size() <= GpuHandler::ComplexMultiplierBatch)
    {
        return -1;
    }

    for (const auto &binSpec : slotAndShift)
    {
        const int slot = binSpec.first;
        const auto slotIndex = static_cast<size_t>(slot);
        if (slot < 0 || m_slotStates.size() <= slotIndex || m_slotStates[slotIndex] != SlotDevice ||
            m_slotLengths[slotIndex] != length)
        {
            return -1;
        }
    }
    if (m_slotPoolStrideFloats != 2UL * length)
    {
        return -1;
    }

    cacheDeviceInfo();

    const auto maxPointsPerGroup = static_cast<unsigned int>(m_localMemorySize / (2 * sizeof(float)));
    if (clampPointsPerGroup(maxPointsPerGroup, length) != length || m_complexMultiplierLocalSize == 0)
    {
        return -1;
    }

    cl_mem dA = ensureResidentInput1(input1, length);
    if (dA == nullptr)
    {
        return -1;
    }

    cl_mem work = ensureBuffer(m_batchWorkBuffer, m_batchWorkCapacity, static_cast<size_t>(bins) * 2UL * length);
    if (work == nullptr || m_error != CL_SUCCESS)
    {
        return -1;
    }
    cl_mem absOut = ensureBuffer(m_batchAbsBuffer, m_batchAbsCapacity, static_cast<size_t>(bins) * length);
    if (absOut == nullptr || m_error != CL_SUCCESS)
    {
        return -1;
    }
    cl_mem params = ensureBuffer(m_batchParamsBuffer, m_batchParamsCapacity, static_cast<size_t>(bins) * 2);
    if (params == nullptr || m_error != CL_SUCCESS)
    {
        return -1;
    }

    m_batchParamsHost.resize(static_cast<size_t>(bins) * 2);
    for (unsigned int k = 0; k < bins; k++)
    {
        const auto slotIndex = static_cast<size_t>(slotAndShift[k].first);
        const int shiftRaw = slotAndShift[k].second;
        const auto shift = static_cast<unsigned int>(
            ((shiftRaw % static_cast<int>(length)) + static_cast<int>(length)) % static_cast<int>(length));
        m_batchParamsHost[2UL * k] = static_cast<cl_uint>(slotIndex * length);
        m_batchParamsHost[(2UL * k) + 1] = (length - shift) % length;
    }
    m_error = clEnqueueWriteBuffer(m_queue,
                                   params,
                                   CL_TRUE,
                                   0,
                                   static_cast<size_t>(bins) * 2 * sizeof(cl_uint),
                                   m_batchParamsHost.data(),
                                   0,
                                   nullptr,
                                   nullptr);
    if (m_error != CL_SUCCESS)
    {
        return -1;
    }

    cl_kernel batchKernel = m_gpu.acquisitionKernelList[GpuHandler::ComplexMultiplierBatch];
    m_error = clSetKernelArg(batchKernel, 0, sizeof(cl_mem), reinterpret_cast<const void *>(&dA));
    m_error |= clSetKernelArg(batchKernel, 1, sizeof(cl_mem), reinterpret_cast<const void *>(&m_slotPoolBuffer));
    m_error |= clSetKernelArg(batchKernel, 2, sizeof(cl_mem), reinterpret_cast<const void *>(&work));
    m_error |= clSetKernelArg(batchKernel, 3, sizeof(cl_mem), reinterpret_cast<const void *>(&params));
    m_error |= clSetKernelArg(batchKernel, 4, sizeof(length), &length);
    if (m_error != CL_SUCCESS)
    {
        return -1;
    }

    size_t multiplyLocalSize = std::min(m_complexMultiplierLocalSize, static_cast<size_t>(length));
    const size_t totalElements = static_cast<size_t>(bins) * length;
    m_error = clEnqueueNDRangeKernel(
        m_queue, batchKernel, 1, nullptr, &totalElements, &multiplyLocalSize, 0, nullptr, nullptr);
    if (m_error != CL_SUCCESS)
    {
        return -1;
    }

    if (fftDeviceInPlaceBatch(work, length, bins, direction) != 0)
    {
        return -1;
    }

    cl_kernel absoluteKernel = m_gpu.acquisitionKernelList[GpuHandler::Absolute];
    size_t absoluteLocalSize = m_absoluteLocalSize;
    unsigned int absolutePointsPerGroup = clampPointsPerGroup(static_cast<unsigned int>(absoluteLocalSize), length);
    absoluteLocalSize =
        clampLocalSizeForMinPointsPerItem(static_cast<unsigned int>(absoluteLocalSize), absolutePointsPerGroup, 1);

    clSetKernelArg(absoluteKernel, 0, sizeof(cl_mem), reinterpret_cast<const void *>(&work));
    clSetKernelArg(absoluteKernel, 1, sizeof(cl_mem), reinterpret_cast<const void *>(&absOut));
    clSetKernelArg(absoluteKernel, 2, sizeof(unsigned int), &absolutePointsPerGroup);

    const size_t absoluteGlobalSize = (totalElements / absolutePointsPerGroup) * absoluteLocalSize;
    m_error = clEnqueueNDRangeKernel(
        m_queue, absoluteKernel, 1, nullptr, &absoluteGlobalSize, &absoluteLocalSize, 0, nullptr, nullptr);
    if (m_error != CL_SUCCESS)
    {
        return -1;
    }

    output->resize(totalElements);
    m_error = clEnqueueReadBuffer(
        m_queue, absOut, CL_TRUE, 0, totalElements * sizeof(float), output->data(), 0, nullptr, nullptr);
    return (m_error == CL_SUCCESS) ? 0 : -1;
}
