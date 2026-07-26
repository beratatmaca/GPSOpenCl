#include "GPSOpenClFileSink.h"
#include <cstdint>

namespace GPSOpenCl
{
FileSink::FileSink(const std::string &outputFilePath)
{
    m_file.open(outputFilePath, std::ios::binary | std::ios::app);
}

FileSink::~FileSink()
{
    if (m_file.is_open())
    {
        m_file.close();
    }
}

void FileSink::publish(const std::string &identifier, const void *data, size_t size)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_file.is_open()) return;

    uint32_t nameLen = static_cast<uint32_t>(identifier.size());
    uint32_t dataLen = static_cast<uint32_t>(size);

    m_file.write(reinterpret_cast<const char *>(&nameLen), sizeof(nameLen));
    m_file.write(identifier.data(), nameLen);
    m_file.write(reinterpret_cast<const char *>(&dataLen), sizeof(dataLen));
    m_file.write(reinterpret_cast<const char *>(data), size);
}
}
