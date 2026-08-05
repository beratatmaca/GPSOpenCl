#include "GPSOpenClGPUHandler.h"

#include <cstdio>
#include <iostream>
#include <memory>

using namespace GPSOpenCl;

GpuHandler::GpuHandler() : m_context(nullptr), m_device(nullptr), m_localMemorySize(0), m_platform(nullptr), m_error(0)
{
}

GpuHandler::~GpuHandler()
{
    for (auto *kernel : m_acquisitionKernelList)
    {
        if (kernel != nullptr)
        {
            clReleaseKernel(kernel);
        }
    }
    m_acquisitionKernelList.clear();

    for (auto *program : m_programList)
    {
        if (program != nullptr)
        {
            clReleaseProgram(program);
        }
    }
    m_programList.clear();

    if (m_context != nullptr)
    {
        clReleaseContext(m_context);
        m_context = nullptr;
    }
}

int GpuHandler::createDevice()
{
    cl_uint numOfPlatforms = 0;
    cl_uint numOfDevices = 0;

    m_error = clGetPlatformIDs(0, nullptr, &numOfPlatforms);
    if (m_error < 0 || numOfPlatforms == 0)
    {
        std::cout << "[INFO] No OpenCL platform driver detected. Using C++ CPU software compute mode." << '\n';
        return m_error < 0 ? m_error : -1;
    }

    std::vector<cl_platform_id> platforms(numOfPlatforms);
    clGetPlatformIDs(numOfPlatforms, platforms.data(), nullptr);

    for (cl_platform_id platform : platforms)
    {
        m_error = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &m_device, &numOfDevices);
        if (m_error == CL_SUCCESS && numOfDevices > 0)
        {
            m_platform = platform;
            break;
        }
    }

    if (m_platform == nullptr)
    {
        for (cl_platform_id platform : platforms)
        {
            m_error = clGetDeviceIDs(platform, CL_DEVICE_TYPE_CPU, 1, &m_device, &numOfDevices);
            if (m_error == CL_SUCCESS && numOfDevices > 0)
            {
                m_platform = platform;
                break;
            }
        }
    }

    if (m_platform == nullptr)
    {
        std::cout << "[INFO] No OpenCL GPU/CPU device detected on any platform. Using C++ CPU software compute mode."
                  << '\n';
        return m_error < 0 ? m_error : -1;
    }

    m_context = clCreateContext(nullptr, 1, &m_device, nullptr, nullptr, &m_error);
    if (m_error < 0)
    {
        std::cout << "Couldn't create a context" << '\n';
    }

    return m_error;
}

int GpuHandler::buildProgram()
{
    size_t programSize = 0;
    size_t logSize = 0;

    for (int i = 0; i < GPSOpenClProgramCount; i++)
    {
        const std::string filename = m_programCharList[i];
        const std::vector<std::string> candidatePaths = {filename,
                                                         "../" + filename,
                                                         "../../" + filename,
                                                         "Kernels/" + filename,
                                                         "../Kernels/" + filename,
                                                         "../../Kernels/" + filename};

        std::unique_ptr<FILE, decltype(&fclose)> programHandle(nullptr, &fclose);
        for (const auto &path : candidatePaths)
        {
            programHandle.reset(fopen(path.c_str(), "r"));
            if (programHandle)
            {
                break;
            }
        }

        if (!programHandle)
        {
            std::cout << "Couldn't find the program file: " << filename << '\n';
            return -1;
        }

        fseek(programHandle.get(), 0, SEEK_END);
        programSize = ftell(programHandle.get());
        fseek(programHandle.get(), 0, SEEK_SET);
        std::vector<char> programBuffer(programSize + 1, '\0');
        fread(programBuffer.data(), sizeof(char), programSize, programHandle.get());
        programHandle.reset();

        const char *sourcePtr = programBuffer.data();
        auto *program = clCreateProgramWithSource(m_context, 1, &sourcePtr, &programSize, &m_error);

        m_programList.push_back(program);

        if (m_error < 0)
        {
            std::cout << "Couldn't create the program" << '\n';
            return m_error;
        }

        m_error = clBuildProgram(m_programList[i], 0, nullptr, "-cl-mad-enable -cl-denorms-are-zero", nullptr, nullptr);
        if (m_error < 0)
        {

            clGetProgramBuildInfo(m_programList[i], m_device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &logSize);
            std::vector<char> programLog(logSize + 1, '\0');
            clGetProgramBuildInfo(
                m_programList[i], m_device, CL_PROGRAM_BUILD_LOG, logSize + 1, programLog.data(), nullptr);
            std::cout << programLog.data() << '\n';
            return m_error;
        }
    }

    return m_error;
}

void GpuHandler::getLastErrorAsString() const
{
    std::cout << m_error << '\n';
}

int GpuHandler::initKernels()
{
    for (int i = 0; i < AcquisitionKernelCount; i++)
    {
        auto *kernelChar = m_acquisitionKernelCharList[i].data();
        auto *program = m_programList[GPSOpenClAcquistion];
        auto *kernel = clCreateKernel(program, kernelChar, &m_error);
        if (m_error < 0)
        {
            std::cout << "Couldn't create the kernel" << i << '\n';
            return m_error;
        }

        m_acquisitionKernelList.push_back(kernel);
    }

    return m_error;
}

int GpuHandler::determineLocalMemorySize()
{
    m_error =
        clGetDeviceInfo(m_device, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(m_localMemorySize), &m_localMemorySize, nullptr);
    if (m_error < 0)
    {
        std::cout << "Couldn't determine the local memory size" << '\n';
        getLastErrorAsString();
    }

    return m_error;
}
