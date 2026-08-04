#include "GPSOpenClFileSink.h"
#include <cstdint>
#include <cstring>

namespace GPSOpenCl
{
FileSink::FileSink(const std::string &outputFilePath) : m_queue(1024)
{
    m_file.open(outputFilePath, std::ios::binary | std::ios::app);
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
    SinkMessage message;
    while (m_queue.pop(message))
    {
        if (!m_file.is_open())
        {
            continue;
        }

        auto nameLen = static_cast<uint32_t>(message.identifierLength);
        auto dataLen = static_cast<uint32_t>(message.dataLength);

        m_file.write(reinterpret_cast<const char *>(&nameLen), sizeof(nameLen));
        m_file.write(message.identifier, nameLen);
        m_file.write(reinterpret_cast<const char *>(&dataLen), sizeof(dataLen));
        m_file.write(message.data, static_cast<std::streamsize>(dataLen));
    }
}
}
