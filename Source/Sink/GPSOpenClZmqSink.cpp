#include "Sink/GPSOpenClZmqSink.hpp"
#include <cstring>
#include <filesystem>
#include <iostream>
#include <utility>

namespace GPSOpenCl
{
namespace
{
void ensureIpcDirectoryExists(const std::string &endpoint)
{
    const std::string ipcPrefix = "ipc://";
    if (endpoint.compare(0, ipcPrefix.size(), ipcPrefix) != 0)
    {
        return;
    }
    const std::filesystem::path socketPath(endpoint.substr(ipcPrefix.size()));
    if (socketPath.has_parent_path())
    {
        std::error_code ec;
        std::filesystem::create_directories(socketPath.parent_path(), ec);
    }
}
}

ZmqSink::ZmqSink(std::string endpoint) : m_endpoint(std::move(endpoint)), m_queue(1024)
{
    // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer): zmq_ctx_new must run in the body
    m_context = zmq_ctx_new();

    if (m_context != nullptr)
    {
        m_publisher = zmq_socket(m_context, ZMQ_PUB);
        if (m_publisher != nullptr)
        {
            ensureIpcDirectoryExists(m_endpoint);
            const int lingerMs = 0;
            zmq_setsockopt(m_publisher, ZMQ_LINGER, &lingerMs, sizeof(lingerMs));
            if (zmq_bind(m_publisher, m_endpoint.c_str()) != 0)
            {
                std::cerr << "ZmqSink: bind to " << m_endpoint << " failed: " << zmq_strerror(zmq_errno()) << " -- telemetry will not be published" << '\n';
                zmq_close(m_publisher);
                m_publisher = nullptr;
            }
        }
    }
    m_senderThread = std::thread([this] { senderThreadLoop(); });
}

ZmqSink::~ZmqSink()
{
    m_queue.finish();
    if (m_senderThread.joinable())
    {
        m_senderThread.join();
    }
    if (m_publisher != nullptr)
    {
        zmq_close(m_publisher);
    }
    if (m_context != nullptr)
    {
        zmq_ctx_destroy(m_context);
    }
}

void ZmqSink::publish(const std::string &identifier, const void *data, size_t size)
{
    SinkMessage message;
    if (!message.fill(identifier, data, size))
    {
        return;
    }
    m_queue.tryPush(message);
}

void ZmqSink::senderThreadLoop()
{
    SinkMessage message;
    while (m_queue.pop(message))
    {
        if (m_publisher == nullptr)
        {
            continue;
        }

        int rc = zmq_send(m_publisher, message.identifier, message.identifierLength, ZMQ_SNDMORE);
        if (rc < 0)
        {
            std::cerr << "ZmqSink: identifier frame send failed: " << zmq_strerror(zmq_errno()) << '\n';
            continue;
        }
        rc = zmq_send(m_publisher, message.data, message.dataLength, 0);
        if (rc < 0)
        {
            std::cerr << "ZmqSink: data frame send failed: " << zmq_strerror(zmq_errno()) << '\n';
        }
    }
}
}
