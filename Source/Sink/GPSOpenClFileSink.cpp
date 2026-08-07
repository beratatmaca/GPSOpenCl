#include "Sink/GPSOpenClFileSink.hpp"
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

namespace GPSOpenCl
{
FileSink::FileSink(const std::string &outputFilePath) : m_queue(1024)
{
    m_file.open(outputFilePath, std::ios::binary | std::ios::app);
    if (!m_file.is_open())
    {
        std::cerr << "FileSink: cannot open " << outputFilePath << " for writing -- telemetry log disabled" << '\n';
    }
    m_writerThread = std::thread([this] { writerThreadLoop(); });
}

FileSink::~FileSink()
{
    m_queue.finish();
    if (m_writerThread.joinable())
    {
        m_writerThread.join();
    }
    if (m_file.is_open())
    {
        m_file.close();
    }
}

void FileSink::publish(const std::string &identifier, const void *data, size_t size)
{
    SinkMessage message;
    if (!message.fill(identifier, data, size))
    {
        return;
    }
    m_queue.tryPush(message);
}

void FileSink::writerThreadLoop()
{
    std::vector<char> record;
    bool writeFailureReported = false;

    SinkMessage message;
    while (m_queue.pop(message))
    {
        if (!m_file.is_open())
        {
            continue;
        }

        const auto nameLen = static_cast<uint32_t>(message.identifierLength);
        const auto dataLen = static_cast<uint32_t>(message.dataLength);

        record.clear();
        record.reserve(sizeof(nameLen) + nameLen + sizeof(dataLen) + dataLen);
        record.insert(record.end(),
                      reinterpret_cast<const char *>(&nameLen),
                      reinterpret_cast<const char *>(&nameLen) + sizeof(nameLen));
        record.insert(record.end(), message.identifier, message.identifier + nameLen);
        record.insert(record.end(),
                      reinterpret_cast<const char *>(&dataLen),
                      reinterpret_cast<const char *>(&dataLen) + sizeof(dataLen));
        record.insert(record.end(), message.data, message.data + dataLen);

        m_file.write(record.data(), static_cast<std::streamsize>(record.size()));
        if (!m_file.good())
        {
            if (!writeFailureReported)
            {
                std::cerr << "FileSink: write failed (disk full or I/O error) -- telemetry log stopped to keep the "
                             "record framing intact"
                          << '\n';
                writeFailureReported = true;
            }
            m_file.close();
        }
    }
}
}
