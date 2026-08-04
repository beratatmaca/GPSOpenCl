#include "GPSOpenClGpsSdrSimSource.h"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

namespace GPSOpenCl
{
GpsSdrSimSource::GpsSdrSimSource() = default;

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

    m_samplesPerBlock = 4096;
    if (m_inputConfig.samplingRate > 0.0)
    {
        m_samplesPerBlock = static_cast<size_t>(
            std::round(m_inputConfig.samplingRate / (GPS_CA_CODE_FREQUENCY_HZ / GPS_CA_CODE_LENGTH)));
    }
    m_byteBuffer.resize(m_samplesPerBlock * 2 * sizeof(int8_t));

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
        const std::string cmdWithNewline = command + "\n";
        const ssize_t written = write(m_ctrlFd, cmdWithNewline.c_str(), cmdWithNewline.length());
        return written > 0;
    }
    return false;
}

bool GpsSdrSimSource::readBlock(ComplexFloatVector &outputSamples, SourceOutput &telemetry)
{
    const size_t samplesPerBlock = m_samplesPerBlock;
    const size_t bytesNeeded = samplesPerBlock * 2 * sizeof(int8_t);
    if (m_byteBuffer.size() < bytesNeeded)
    {
        m_byteBuffer.resize(bytesNeeded);
    }

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
    int idleWaits = 0;
    while (totalRead < bytesNeeded && idleWaits < 50)
    {
        const ssize_t res = read(m_dataFd, m_byteBuffer.data() + totalRead, bytesNeeded - totalRead);
        if (res > 0)
        {
            totalRead += res;
            continue;
        }
        if (res < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
        {
            break;
        }
        if (res == 0)
        {
            usleep(100'000);
        }
        else
        {
            struct pollfd pfd{};
            pfd.fd = m_dataFd;
            pfd.events = POLLIN;
            poll(&pfd, 1, 100);
        }
        idleWaits++;
    }

    if (totalRead < bytesNeeded)
    {
        m_underrunCount++;
        return false;
    }

    outputSamples.resize(samplesPerBlock);
    for (size_t i = 0; i < samplesPerBlock; i++)
    {
        auto re = static_cast<float>(m_byteBuffer[2 * i]);
        auto im = static_cast<float>(m_byteBuffer[(2 * i) + 1]);
        outputSamples[i] = std::complex<float>(re, im);
    }

    telemetry.structVersion = STRUCT_VERSION_1;
    telemetry.blockIndex = m_blockIndex;
    telemetry.timestamp = static_cast<double>(m_blockIndex) * 0.001;
    m_blockIndex++;
    telemetry.fifoUnderrunCount = m_underrunCount;
    telemetry.fifoOverrunCount = m_overrunCount;

    if (m_sink)
    {
        m_sink->publishSourceOutput(telemetry);
    }

    return true;
}
}
