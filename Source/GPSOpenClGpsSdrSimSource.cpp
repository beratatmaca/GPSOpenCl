#include "GPSOpenClGpsSdrSimSource.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <vector>

namespace GPSOpenCl
{
GpsSdrSimSource::GpsSdrSimSource()
{
}

GpsSdrSimSource::~GpsSdrSimSource()
{
    if (m_dataFd >= 0)
    {
        close(m_dataFd);
    }
    if (m_ctrlFd >= 0)
    {
        close(m_ctrlFd);
    }
}

bool GpsSdrSimSource::initialize(const SourceInput &input)
{
    m_inputConfig = input;
    m_dataFifoPath = (strlen(input.fifoPath) > 0) ? input.fifoPath : "/tmp/gpsopencl/sim_data.fifo";
    m_ctrlFifoPath = "/tmp/gpsopencl/sim_ctrl.fifo";

    mkdir("/tmp/gpsopencl", 0755);
    mkfifo(m_dataFifoPath.c_str(), 0666);
    mkfifo(m_ctrlFifoPath.c_str(), 0666);

    m_dataFd = open(m_dataFifoPath.c_str(), O_RDONLY | O_NONBLOCK);
    m_ctrlFd = open(m_ctrlFifoPath.c_str(), O_RDWR | O_NONBLOCK);

    return true;
}

bool GpsSdrSimSource::sendControlCommand(const std::string &command)
{
    if (m_ctrlFd < 0)
    {
        m_ctrlFd = open(m_ctrlFifoPath.c_str(), O_RDWR | O_NONBLOCK);
    }
    if (m_ctrlFd >= 0)
    {
        std::string cmdWithNewline = command + "\n";
        ssize_t written = write(m_ctrlFd, cmdWithNewline.c_str(), cmdWithNewline.length());
        return written > 0;
    }
    return false;
}

bool GpsSdrSimSource::readBlock(ComplexFloatVector &outputSamples, SourceOutput &telemetry)
{
    size_t samplesPerBlock = 4096;
    size_t bytesNeeded = samplesPerBlock * 2 * sizeof(int8_t);
    std::vector<int8_t> buffer(bytesNeeded);

    if (m_dataFd < 0)
    {
        m_dataFd = open(m_dataFifoPath.c_str(), O_RDONLY | O_NONBLOCK);
        if (m_dataFd < 0)
        {
            m_underrunCount++;
            return false;
        }
    }

    size_t totalRead = 0;
    int retryCount = 0;
    while (totalRead < bytesNeeded && retryCount < 100)
    {
        ssize_t res = read(m_dataFd, buffer.data() + totalRead, bytesNeeded - totalRead);
        if (res > 0)
        {
            totalRead += res;
        }
        else
        {
            usleep(100);
            retryCount++;
        }
    }

    if (totalRead < bytesNeeded)
    {
        m_underrunCount++;
        return false;
    }

    outputSamples.resize(samplesPerBlock);
    for (size_t i = 0; i < samplesPerBlock; i++)
    {
        float re = static_cast<float>(buffer[2 * i]);
        float im = static_cast<float>(buffer[2 * i + 1]);
        outputSamples[i] = std::complex<float>(re, im);
    }

    telemetry.structVersion = STRUCT_VERSION_1;
    telemetry.blockIndex = m_blockIndex++;
    telemetry.timestamp = static_cast<double>(m_blockIndex) * 0.001;
    telemetry.fifoUnderrunCount = m_underrunCount;
    telemetry.fifoOverrunCount = m_overrunCount;

    if (m_sink)
    {
        m_sink->publishSourceOutput(telemetry);
    }

    return true;
}
}
