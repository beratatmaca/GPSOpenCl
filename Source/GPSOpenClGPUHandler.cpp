#include "GPSOpenClGPUHandler.h"

#include <cstdio>
#include <iostream>
#include <memory>

using namespace GPSOpenCl;

GpuHandler::GpuHandler() = default;

GpuHandler::~GpuHandler()
{
    for (auto *kernel : acquisitionKernelList)
    {
        if (kernel != nullptr)
        {
            clReleaseKernel(kernel);
        }
    }
    acquisitionKernelList.clear();

    for (auto *program : programList)
    {
        if (program != nullptr)
        {
            clReleaseProgram(program);
        }
    }
    programList.clear();

    if (context != nullptr)
    {
        clReleaseContext(context);
        context = nullptr;
    }
}

int GpuHandler::createDevice()
{
    cl_uint numOfPlatforms = 0;
    cl_uint numOfDevices = 0;

    m_lastError = clGetPlatformIDs(0, nullptr, &numOfPlatforms);
    if (m_lastError < 0 || numOfPlatforms == 0)
    {
        std::cout << "[INFO] No OpenCL m_platform driver detected. Using C++ CPU software compute mode." << '\n';
        return m_lastError < 0 ? m_lastError : -1;
    }

    std::vector<cl_platform_id> platforms(numOfPlatforms);
    clGetPlatformIDs(numOfPlatforms, platforms.data(), nullptr);

    for (cl_platform_id candidatePlatform : platforms)
    {
        m_lastError = clGetDeviceIDs(candidatePlatform, CL_DEVICE_TYPE_GPU, 1, &device, &numOfDevices);
        if (m_lastError == CL_SUCCESS && numOfDevices > 0)
        {
            m_platform = candidatePlatform;
            break;
        }
    }

    if (m_platform == nullptr)
    {
        for (cl_platform_id candidatePlatform : platforms)
        {
            m_lastError = clGetDeviceIDs(candidatePlatform, CL_DEVICE_TYPE_CPU, 1, &device, &numOfDevices);
            if (m_lastError == CL_SUCCESS && numOfDevices > 0)
            {
                m_platform = candidatePlatform;
                break;
            }
        }
    }

    if (m_platform == nullptr)
    {
        std::cout << "[INFO] No OpenCL GPU/CPU device detected on any m_platform. Using C++ CPU software compute mode."
                  << '\n';
        return m_lastError < 0 ? m_lastError : -1;
    }

    context = clCreateContext(nullptr, 1, &device, nullptr, nullptr, &m_lastError);
    if (m_lastError < 0)
    {
        std::cout << "Couldn't create a context" << '\n';
    }

    return m_lastError;
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
            // NOLINTNEXTLINE(cppcoreguidelines-owning-memory): unique_ptr with fclose deleter owns the handle
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
        auto *program = clCreateProgramWithSource(context, 1, &sourcePtr, &programSize, &m_lastError);

        programList.push_back(program);

        if (m_lastError < 0)
        {
            std::cout << "Couldn't create the program" << '\n';
            return m_lastError;
        }

        m_lastError =
            clBuildProgram(programList[i], 0, nullptr, "-cl-mad-enable -cl-denorms-are-zero", nullptr, nullptr);
        if (m_lastError < 0)
        {

            clGetProgramBuildInfo(programList[i], device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &logSize);
            std::vector<char> programLog(logSize + 1, '\0');
            clGetProgramBuildInfo(
                programList[i], device, CL_PROGRAM_BUILD_LOG, logSize + 1, programLog.data(), nullptr);
            std::cout << programLog.data() << '\n';
            return m_lastError;
        }
    }

    return m_lastError;
}

void GpuHandler::getLastErrorAsString() const
{
    std::cout << m_lastError << '\n';
}

int GpuHandler::initKernels()
{
    for (int i = 0; i < AcquisitionKernelCount; i++)
    {
        auto *kernelChar = m_acquisitionKernelCharList[i].data();
        auto *program = programList[GPSOpenClAcquistion];
        auto *kernel = clCreateKernel(program, kernelChar, &m_lastError);
        if (m_lastError < 0)
        {
            std::cout << "Couldn't create the kernel" << i << '\n';
            return m_lastError;
        }

        acquisitionKernelList.push_back(kernel);
    }

    return m_lastError;
}

int GpuHandler::determineLocalMemorySize()
{
    m_lastError = clGetDeviceInfo(device, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(localMemorySize), &localMemorySize, nullptr);
    if (m_lastError < 0)
    {
        std::cout << "Couldn't determine the local memory size" << '\n';
        getLastErrorAsString();
    }

    return m_lastError;
}
