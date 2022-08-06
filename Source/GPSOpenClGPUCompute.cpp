#include "GPSOpenClGPUCompute.h"

#include <cmath>
#include <cstring>
#include <iostream>

using namespace GPSOpenCl;

/**
 * @brief Construct a new GPSOpenCl::Compute::Compute object
 *
 */
Compute::Compute()
{
    m_error = 0;

    m_gpu.createDevice();
    m_gpu.buildProgram();
    m_gpu.initKernels();

    m_queue = clCreateCommandQueueWithProperties(m_gpu.m_context, m_gpu.m_device, NULL, &m_error);
    if (m_error < 0)
    {
        std::cout << "Couldn't create a command queue" << std::endl;
        m_gpu.getLastErrorAsString();
    }
    else
    {
    }
}

/**
 * @brief Destroy the GPSOpenCl::Compute::Compute object
 *
 */
Compute::~Compute()
{
}

/**
 * @brief
 *
 * @param input
 * @param output
 * @param direction
 * @return int
 */
int Compute::fft(const ComplexFloatVector input, ComplexFloatVector *output, FFTDirectionType direction)
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

    for (unsigned int j = 0; j < length; j++)
    {
        float realVal = std::real(input.at(j));
        float imagVal = std::imag(input.at(j));

        m_allocatedMemory[2 * j] = realVal;
        m_allocatedMemory[2 * j + 1] = imagVal;
    }

    cl_mem dataBuffer = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                                       2 * length * sizeof(float), m_allocatedMemory, &m_error);

    if (m_error < 0)
    {
        std::cout << "Couldn't create a buffer. Error: " << std::endl;
        m_gpu.getLastErrorAsString();
        return m_error;
    }

    m_error = clGetKernelWorkGroupInfo(initKernel, m_gpu.m_device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(local_size),
                                       &local_size, NULL);
    if (m_error < 0)
    {
        std::cout << "Couldn't find the maximum work-group size" << std::endl;
        m_gpu.getLastErrorAsString();
        return m_error;
    }
    else
    {
    }

    local_size = (int)pow(2, trunc(log2(local_size)));

    m_error = m_gpu.determineLocalMemorySize();
    if (CL_SUCCESS != m_error)
    {
        std::cout << "An error detected while determinin local memory size" << std::endl;
        return m_error;
    }
    localMemorySize = m_gpu.m_localMemorySize;

    points_per_group = localMemorySize / (2 * sizeof(float));
    if (points_per_group > length)
    {
        points_per_group = length;
    }

    m_error = clSetKernelArg(initKernel, 0, sizeof(cl_mem), &dataBuffer);
    m_error |= clSetKernelArg(initKernel, 1, localMemorySize, NULL);
    m_error |= clSetKernelArg(initKernel, 2, sizeof(points_per_group), &points_per_group);
    m_error |= clSetKernelArg(initKernel, 3, sizeof(length), &length);
    m_error |= clSetKernelArg(initKernel, 4, sizeof(dir), &dir);
    if (m_error < 0)
    {
        std::cout << "Couldn't set a kernel argument" << std::endl;
        m_gpu.getLastErrorAsString();
        return m_error;
    }
    else
    {
    }

    global_size = (length / points_per_group) * local_size;
    m_error = clEnqueueNDRangeKernel(m_queue, initKernel, 1, NULL, &global_size, &local_size, 0, NULL, NULL);
    if (m_error < 0)
    {
        std::cout << "Couldn't enqueue the initial kernel" << std::endl;
        m_gpu.getLastErrorAsString();
        return m_error;
    }

    if (length > points_per_group)
    {
        m_error = clSetKernelArg(stageKernel, 0, sizeof(cl_mem), &dataBuffer);
        m_error |= clSetKernelArg(stageKernel, 2, sizeof(points_per_group), &points_per_group);
        m_error |= clSetKernelArg(stageKernel, 3, sizeof(dir), &dir);
        if (m_error < 0)
        {
            std::cout << "Couldn't set a kernel argument" << std::endl;
            m_gpu.getLastErrorAsString();
            return m_error;
        }
        else
        {
        }

        for (stage = 2; stage <= length / points_per_group; stage <<= 1)
        {
            clSetKernelArg(stageKernel, 1, sizeof(stage), &stage);
            m_error = clEnqueueNDRangeKernel(m_queue, stageKernel, 1, NULL, &global_size, &local_size, 0, NULL, NULL);
            if (m_error < 0)
            {
                std::cout << "Couldn't enqueue the stage kernel" << std::endl;
                m_gpu.getLastErrorAsString();
                return m_error;
            }
            else
            {
            }
        }
    }

    if (dir < 0)
    {
        m_error = clSetKernelArg(scaleKernel, 0, sizeof(cl_mem), &dataBuffer);
        m_error |= clSetKernelArg(scaleKernel, 1, sizeof(points_per_group), &points_per_group);
        m_error |= clSetKernelArg(scaleKernel, 2, sizeof(length), &length);
        if (m_error < 0)
        {
            std::cout << "Couldn't set a kernel argument" << std::endl;
            m_gpu.getLastErrorAsString();
            return m_error;
        }
        else
        {
        }

        m_error = clEnqueueNDRangeKernel(m_queue, scaleKernel, 1, NULL, &global_size, &local_size, 0, NULL, NULL);
        if (m_error < 0)
        {
            std::cout << "Couldn't enqueue the initial kernel" << std::endl;
            m_gpu.getLastErrorAsString();
            return m_error;
        }
        else
        {
        }
    }

    m_error = clEnqueueReadBuffer(m_queue, dataBuffer, CL_TRUE, 0, 2 * length * sizeof(float), m_allocatedMemory, 0,
                                  NULL, NULL);
    if (m_error < 0)
    {
        std::cout << "Couldn't read the buffer" << std::endl;
        m_gpu.getLastErrorAsString();
        return m_error;
    }
    else
    {
    }

    for (unsigned int j = 0; j < length; j++)
    {
        float realVal = m_allocatedMemory[2 * j];
        float imagVal = m_allocatedMemory[2 * j + 1];

        output->push_back(std::complex<float>(realVal, imagVal));
    }

    if (CL_SUCCESS != m_error)
    {
        std::cout << "An error detected while capturing result" << std::endl;
        return m_error;
    }

    return m_error;
}

int Compute::complexMultiplier(const ComplexFloatVector input1, const ComplexFloatVector input2,
                               ComplexFloatVector *output)
{
    size_t global_size = 0;
    size_t local_size = 0;
    unsigned int length = static_cast<unsigned int>(input1.size());
    cl_mem d_a;
    cl_mem d_b;
    cl_mem d_c;
    cl_ulong localMemorySize;
    unsigned int points_per_group = 0;

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
    d_b = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE, 2 * length * sizeof(float), NULL, &m_error);
    d_c = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE, 2 * length * sizeof(float), NULL, &m_error);

    m_error =
        clEnqueueWriteBuffer(m_queue, d_a, CL_TRUE, 0, 2 * length * sizeof(float), m_allocatedMemory, 0, NULL, NULL);
    m_error = clEnqueueWriteBuffer(m_queue, d_b, CL_TRUE, 0, 2 * length * sizeof(float), &m_allocatedMemory[2 * length],
                                   0, NULL, NULL);

    m_error = clGetKernelWorkGroupInfo(complexMultiplierKernel, m_gpu.m_device, CL_KERNEL_WORK_GROUP_SIZE,
                                       sizeof(local_size), &local_size, NULL);
    local_size = (int)pow(2, trunc(log2(local_size)));

    m_error = m_gpu.determineLocalMemorySize();
    localMemorySize = m_gpu.m_localMemorySize;

    points_per_group = localMemorySize / (2 * sizeof(float));
    if (points_per_group > length)
    {
        points_per_group = length;
    }

    m_error = clSetKernelArg(complexMultiplierKernel, 0, sizeof(cl_mem), &d_a);
    m_error |= clSetKernelArg(complexMultiplierKernel, 1, sizeof(cl_mem), &d_b);
    m_error |= clSetKernelArg(complexMultiplierKernel, 2, sizeof(cl_mem), &d_c);
    m_error |= clSetKernelArg(complexMultiplierKernel, 3, sizeof(unsigned int), &points_per_group);

    global_size = (length / points_per_group) * local_size;
    m_error =
        clEnqueueNDRangeKernel(m_queue, complexMultiplierKernel, 1, NULL, &global_size, &local_size, 0, NULL, NULL);

    clEnqueueReadBuffer(m_queue, d_c, CL_TRUE, 0, 2 * length * sizeof(float), m_allocatedMemory, 0, NULL, NULL);

    for (unsigned int j = 0; j < length; j++)
    {
        float realVal = m_allocatedMemory[2 * j];
        float imagVal = m_allocatedMemory[2 * j + 1];

        output->push_back(std::complex<float>(realVal, imagVal));
    }

    return m_error;
}

int Compute::absolute(const ComplexFloatVector input1, FloatVector *output)
{
    size_t global_size = 0;
    size_t local_size = 0;
    unsigned int length = static_cast<unsigned int>(input1.size());
    cl_mem d_a;
    cl_mem d_c;
    cl_ulong localMemorySize;
    unsigned int points_per_group = 0;

    for (unsigned int j = 0; j < length; j++)
    {
        float realVal = std::real(input1.at(j));
        float imagVal = std::imag(input1.at(j));

        m_allocatedMemory[2 * j] = realVal;
        m_allocatedMemory[2 * j + 1] = imagVal;
    }

    cl_kernel absoluteKernel = m_gpu.m_acquisitionKernelList[GpuHandler::Absolute];
    d_a = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE, 2 * length * sizeof(float), NULL, &m_error);
    d_c = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE, 2 * length * sizeof(float), NULL, &m_error);

    m_error =
        clEnqueueWriteBuffer(m_queue, d_a, CL_TRUE, 0, 2 * length * sizeof(float), m_allocatedMemory, 0, NULL, NULL);

    m_error = clGetKernelWorkGroupInfo(absoluteKernel, m_gpu.m_device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(local_size),
                                       &local_size, NULL);
    local_size = (int)pow(2, trunc(log2(local_size)));

    m_error = m_gpu.determineLocalMemorySize();
    localMemorySize = m_gpu.m_localMemorySize;

    points_per_group = localMemorySize / (2 * sizeof(float));
    if (points_per_group > length)
    {
        points_per_group = length;
    }

    m_error = clSetKernelArg(absoluteKernel, 0, sizeof(cl_mem), &d_a);
    m_error |= clSetKernelArg(absoluteKernel, 1, sizeof(cl_mem), &d_c);
    m_error |= clSetKernelArg(absoluteKernel, 2, sizeof(unsigned int), &points_per_group);

    global_size = length;
    m_error = clEnqueueNDRangeKernel(m_queue, absoluteKernel, 1, NULL, &global_size, &local_size, 0, NULL, NULL);

    clEnqueueReadBuffer(m_queue, d_c, CL_TRUE, 0, 2 * length * sizeof(float), m_allocatedMemory, 0, NULL, NULL);

    for (unsigned int j = 0; j < length; j++)
    {
        float value = m_allocatedMemory[2 * j];

        output->push_back(value);
    }

    return m_error;
}

int Compute::sum(const FloatVector input, float *sumValue)
{
    size_t global_size = 0;
    size_t local_size = 0;
    cl_mem d_input;
    cl_mem d_sumValue;
    cl_ulong localMemorySize;
    unsigned int length = static_cast<unsigned int>(input.size());
    unsigned int points_per_group = 0;
    float tmpSumValue = 0.0f;

    for (unsigned int j = 0; j < length; j++)
    {
        m_allocatedMemory[j] = input.at(j);
    }

    cl_kernel sumKernel = m_gpu.m_acquisitionKernelList[GpuHandler::Sum];

    m_error = clGetKernelWorkGroupInfo(sumKernel, m_gpu.m_device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(local_size),
                                       &local_size, NULL);
    local_size = (int)pow(2, trunc(log2(local_size)));

    m_error = m_gpu.determineLocalMemorySize();
    localMemorySize = m_gpu.m_localMemorySize;

    points_per_group = localMemorySize / sizeof(float);
    if (points_per_group > length)
    {
        points_per_group = length;
    }

    d_input = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE, length * sizeof(float), NULL, &m_error);
    d_sumValue = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE, sizeof(float), NULL, &m_error);

    global_size = length;
    size_t computedSize = 0;
    do
    {
        m_error = clEnqueueWriteBuffer(m_queue, d_input, CL_TRUE, 0, local_size * sizeof(float),
                                       &m_allocatedMemory[computedSize], 0, NULL, NULL);

        m_error = clSetKernelArg(sumKernel, 0, sizeof(cl_mem), &d_input);
        m_error = clSetKernelArg(sumKernel, 1, sizeof(cl_mem), &d_sumValue);
        m_error = clSetKernelArg(sumKernel, 2, sizeof(float) * local_size, NULL);

        m_error = clEnqueueNDRangeKernel(m_queue, sumKernel, 1, NULL, &global_size, &local_size, 0, NULL, NULL);
        m_error = clEnqueueReadBuffer(m_queue, d_sumValue, CL_TRUE, 0, sizeof(float), &tmpSumValue, 0, NULL, NULL);

        computedSize += local_size;
        *sumValue += tmpSumValue;
    } while (computedSize < global_size);

    return m_error;
}

int Compute::ncoMultiplication(const ComplexFloatVector input, const FloatVector phaseVector,
                               ComplexFloatVector *output)
{
    size_t global_size = 0;
    size_t local_size = 0;
    unsigned int points_per_group = 0;
    unsigned int length = input.size();
    cl_ulong localMemorySize;

    cl_kernel ncoMultiplicationKernel = m_gpu.m_trackingKernelList[GpuHandler::NCOMultiplicate];

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
                                       2 * length * sizeof(float), m_allocatedMemory, &m_error);

    cl_mem phaseBuffer = clCreateBuffer(m_gpu.m_context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                                        length * sizeof(float), &m_allocatedMemory[2 * length], &m_error);

    m_error = clGetKernelWorkGroupInfo(ncoMultiplicationKernel, m_gpu.m_device, CL_KERNEL_WORK_GROUP_SIZE,
                                       sizeof(local_size), &local_size, NULL);
    local_size = (int)pow(2, trunc(log2(local_size)));

    m_error = m_gpu.determineLocalMemorySize();
    localMemorySize = m_gpu.m_localMemorySize;

    points_per_group = localMemorySize / (2 * sizeof(float));
    points_per_group = (points_per_group > length) ? length : points_per_group;

    m_error = clSetKernelArg(ncoMultiplicationKernel, 0, sizeof(cl_mem), &dataBuffer);
    m_error = clSetKernelArg(ncoMultiplicationKernel, 1, sizeof(cl_mem), &phaseBuffer);
    m_error = clSetKernelArg(ncoMultiplicationKernel, 2, localMemorySize, NULL);
    m_error = clSetKernelArg(ncoMultiplicationKernel, 3, sizeof(unsigned int), &points_per_group);

    global_size = (length / points_per_group) * local_size;
    m_error =
        clEnqueueNDRangeKernel(m_queue, ncoMultiplicationKernel, 1, NULL, &global_size, &local_size, 0, NULL, NULL);

    m_error = clEnqueueReadBuffer(m_queue, dataBuffer, CL_TRUE, 0, 2 * length * sizeof(float), m_allocatedMemory, 0,
                                  NULL, NULL);

    for (unsigned int j = 0; j < length; j++)
    {
        float realVal = m_allocatedMemory[2 * j];
        float imagVal = m_allocatedMemory[2 * j + 1];

        output->push_back(std::complex<float>(realVal, imagVal));
    }

    return m_error;
}
