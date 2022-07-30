#include "GPSOpenClGPUHandler.h"

#include <iostream>

using namespace GPSOpenCl;

/**
 * @brief Construct a new GpuHandler::GpuHandler object
 *
 */
GpuHandler::GpuHandler()
{
}

/**
 * @brief Destroy the GpuHandler::GpuHandler object
 *
 */
GpuHandler::~GpuHandler()
{
}

/**
 * @brief
 *
 * @return int
 */
int GpuHandler::createDevice()
{
    cl_platform_id *platforms;
    cl_uint numOfPlatforms;
    cl_uint numOfDevices;

    m_error = clGetPlatformIDs(1, NULL, &numOfPlatforms);
    if (m_error < 0)
    {
        std::cout << "Couldn't identify a platform" << std::endl;
        return m_error;
    }
    else
    {
        if (numOfPlatforms > 0)
        {
            platforms = (cl_platform_id *)malloc(sizeof(cl_platform_id) * numOfPlatforms);
            clGetPlatformIDs(numOfPlatforms, platforms, NULL);
            m_platform = platforms[0];
            free(platforms);
        }
    }

    m_error = clGetDeviceIDs(m_platform, CL_DEVICE_TYPE_GPU, 1, &m_device, &numOfDevices);
    if (m_error == CL_DEVICE_NOT_FOUND)
    {
        m_error = clGetDeviceIDs(m_platform, CL_DEVICE_TYPE_CPU, 1, &m_device, NULL);
    }
    else
    {
    }

    if (m_error < 0)
    {
        std::cout << "Couldn't access any devices" << std::endl;
        return m_error;
    }
    else
    {
    }

    m_context = clCreateContext(NULL, 1, &m_device, NULL, NULL, &m_error);
    if (m_error < 0)
    {
        std::cout << "Couldn't create a context" << std::endl;
    }

    return m_error;
}

/**
 * @brief
 *
 * @return int
 */
int GpuHandler::buildProgram()
{
    FILE *program_handle;
    char *program_buffer, *program_log;
    size_t program_size, log_size;

    for (int i = 0; i < GPSOpenClProgramCount; i++)
    {
        // Read program file and place content into buffer
        program_handle = fopen(ProgramCharList[i].data(), "r");

        if (program_handle == NULL)
        {
            std::cout << "Couldn't find the program file" << std::endl;
            return -1;
        }
        else
        {
        }

        // Read Program File
        fseek(program_handle, 0, SEEK_END);
        program_size = ftell(program_handle);
        rewind(program_handle);
        program_buffer = (char *)malloc(program_size + 1);
        program_buffer[program_size] = '\0';
        fread(program_buffer, sizeof(char), program_size, program_handle);
        fclose(program_handle);

        // Create program file
        auto program = clCreateProgramWithSource(m_context, 1, (const char **)&program_buffer, &program_size, &m_error);

        m_programList.push_back(program);

        if (m_error < 0)
        {
            std::cout << "Couldn't create the program" << std::endl;
            free(program_buffer);
            return m_error;
        }
        else
        {
        }

        free(program_buffer);

        /* Build program */
        m_error = clBuildProgram(m_programList[i], 0, NULL, NULL, NULL, NULL);
        if (m_error < 0)
        {

            /* Find size of log and print to std output */
            clGetProgramBuildInfo(m_programList[i], m_device, CL_PROGRAM_BUILD_LOG, 0, NULL, &log_size);
            program_log = (char *)malloc(log_size + 1);
            program_log[log_size] = '\0';
            clGetProgramBuildInfo(m_programList[i], m_device, CL_PROGRAM_BUILD_LOG, log_size + 1, program_log, NULL);
            std::cout << program_log << std::endl;
            free(program_log);
            return m_error;
        }
    }

    return m_error;
}

/**
 * @brief
 *
 */
void GpuHandler::getLastErrorAsString()
{
    // @todo convert error code to string
    std::cout << m_error << std::endl;
}

/**
 * @brief
 *
 * @return int
 */
int GpuHandler::initKernels()
{
    for (int i = 0; i < AcquisitionKernelCount; i++)
    {
        auto kernelChar = GPSOpenClAcquisitionKernelCharList[i].data();
        auto program = m_programList[GPSOpenClAcquistion];
        auto kernel = clCreateKernel(program, kernelChar, &m_error);
        if (m_error < 0)
        {
            std::cout << "Couldn't create the kernel" << i << std::endl;
            return m_error;
        }
        else
        {
            m_acquisitionKernelList.push_back(kernel);
        }
    }

    for (int i = 0; i < TrackingKernelCount; i++)
    {
        auto kernelChar = GPSOpenClTrackingKernelCharList[i].data();
        auto program = m_programList[GPSOpenClTracking];
        auto kernel = clCreateKernel(program, kernelChar, &m_error);
        if (m_error < 0)
        {
            std::cout << "Couldn't create the kernel" << i << std::endl;
            return m_error;
        }
        else
        {
            m_trackingKernelList.push_back(kernel);
        }
    }

    return m_error;
}

/**
 * @brief
 *
 * @return int
 */
int GpuHandler::determineLocalMemorySize(void)
{
    m_error = clGetDeviceInfo(m_device, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(m_localMemorySize), &m_localMemorySize, NULL);
    if (m_error < 0)
    {
        std::cout << "Couldn't determine the local memory size" << std::endl;
        getLastErrorAsString();
    }
    else
    {
    }

    return m_error;
}
