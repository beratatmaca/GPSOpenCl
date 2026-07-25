#include "GPSOpenClGPUHandler.h"

#include <iostream>

using namespace GPSOpenCl;





GpuHandler::GpuHandler() : m_context(nullptr), m_device(nullptr), m_localMemorySize(0), m_platform(nullptr), m_error(0)
{
}





GpuHandler::~GpuHandler()
{
    for (auto kernel : m_acquisitionKernelList)
    {
        if (kernel)
        {
            clReleaseKernel(kernel);
        }
    }
    m_acquisitionKernelList.clear();

    for (auto kernel : m_trackingKernelList)
    {
        if (kernel)
        {
            clReleaseKernel(kernel);
        }
    }
    m_trackingKernelList.clear();

    for (auto program : m_programList)
    {
        if (program)
        {
            clReleaseProgram(program);
        }
    }
    m_programList.clear();

    if (m_context)
    {
        clReleaseContext(m_context);
        m_context = nullptr;
    }
}






int GpuHandler::createDevice()
{
    cl_platform_id *platforms;
    cl_uint numOfPlatforms = 0;
    cl_uint numOfDevices = 0;

    m_error = clGetPlatformIDs(1, NULL, &numOfPlatforms);
    if (m_error < 0 || numOfPlatforms == 0)
    {
        std::cout << "[INFO] No OpenCL platform driver detected. Using C++ CPU software compute mode." << std::endl;
        return m_error < 0 ? m_error : -1;
    }
    else
    {
        platforms = (cl_platform_id *)malloc(sizeof(cl_platform_id) * numOfPlatforms);
        clGetPlatformIDs(numOfPlatforms, platforms, NULL);
        m_platform = platforms[0];
        free(platforms);
    }

    m_error = clGetDeviceIDs(m_platform, CL_DEVICE_TYPE_GPU, 1, &m_device, &numOfDevices);
    if (m_error == CL_DEVICE_NOT_FOUND || m_error < 0 || numOfDevices == 0)
    {

        m_error = clGetDeviceIDs(m_platform, CL_DEVICE_TYPE_CPU, 1, &m_device, &numOfDevices);
    }

    if (m_error < 0 || numOfDevices == 0)
    {
        std::cout << "[INFO] No OpenCL GPU/CPU device detected. Using C++ CPU software compute mode." << std::endl;
        return m_error < 0 ? m_error : -1;
    }

    m_context = clCreateContext(NULL, 1, &m_device, NULL, NULL, &m_error);
    if (m_error < 0)
    {
        std::cout << "Couldn't create a context" << std::endl;
    }

    return m_error;
}






int GpuHandler::buildProgram()
{
    FILE *program_handle = NULL;
    char *program_buffer, *program_log;
    size_t program_size, log_size;

    for (int i = 0; i < GPSOpenClProgramCount; i++)
    {
        std::string filename = ProgramCharList[i];
        std::vector<std::string> candidatePaths = {
            filename,
            "../" + filename,
            "../../" + filename,
            "Kernels/" + filename,
            "../Kernels/" + filename,
            "../../Kernels/" + filename
        };

        program_handle = NULL;
        for (const auto &path : candidatePaths)
        {
            program_handle = fopen(path.c_str(), "r");
            if (program_handle != NULL)
            {
                break;
            }
        }

        if (program_handle == NULL)
        {
            std::cout << "Couldn't find the program file: " << filename << std::endl;
            return -1;
        }


        fseek(program_handle, 0, SEEK_END);
        program_size = ftell(program_handle);
        rewind(program_handle);
        program_buffer = (char *)malloc(program_size + 1);
        program_buffer[program_size] = '\0';
        fread(program_buffer, sizeof(char), program_size, program_handle);
        fclose(program_handle);


        auto program = clCreateProgramWithSource(m_context, 1, (const char **)&program_buffer, &program_size, &m_error);

        m_programList.push_back(program);

        if (m_error < 0)
        {
            std::cout << "Couldn't create the program" << std::endl;
            free(program_buffer);
            return m_error;
        }

        free(program_buffer);


        m_error = clBuildProgram(m_programList[i], 0, NULL, NULL, NULL, NULL);
        if (m_error < 0)
        {

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





void GpuHandler::getLastErrorAsString()
{
    std::cout << m_error << std::endl;
}






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






int GpuHandler::determineLocalMemorySize(void)
{
    m_error = clGetDeviceInfo(m_device, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(m_localMemorySize), &m_localMemorySize, NULL);
    if (m_error < 0)
    {
        std::cout << "Couldn't determine the local memory size" << std::endl;
        getLastErrorAsString();
    }

    return m_error;
}
