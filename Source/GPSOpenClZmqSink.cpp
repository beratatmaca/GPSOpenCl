#include "GPSOpenClZmqSink.h"
#include <cstring>
#include <iostream>

namespace GPSOpenCl
{
ZmqSink::ZmqSink(const std::string &endpoint) : m_endpoint(endpoint), m_queue(1024)
{
#ifdef GPSOPENCL_ENABLE_ZMQ
    m_context = zmq_ctx_new();
    if (m_context)
    {
        m_publisher = zmq_socket(m_context, ZMQ_PUB);
        if (m_publisher)
        {
            zmq_bind(m_publisher, m_endpoint.c_str());
        }
    }
#endif
    m_senderThread = std::thread([this] { senderThreadLoop(); });
}

ZmqSink::~ZmqSink()
{
    m_queue.finish();
    if (m_senderThread.joinable())
    {
        m_senderThread.join();
    }
#ifdef GPSOPENCL_ENABLE_ZMQ
    if (m_publisher)
    {
        zmq_close(m_publisher);
    }
    if (m_context)
    {
        zmq_ctx_destroy(m_context);
    }
#endif
}

void ZmqSink::publish(const std::string &identifier, const void *data, size_t size)
{
    SinkMessage message;
    message.identifier = identifier;
    message.data.resize(size);
    std::memcpy(message.data.data(), data, size);
    m_queue.tryPush(std::move(message));
}

void ZmqSink::senderThreadLoop()
{
    SinkMessage message;
    while (m_queue.pop(message))
    {
#ifdef GPSOPENCL_ENABLE_ZMQ
        if (!m_publisher) continue;

        int rc = zmq_send(m_publisher, message.identifier.data(), message.identifier.size(), ZMQ_SNDMORE);
        if (rc < 0)
        {
            std::cerr << "ZmqSink: identifier frame send failed: " << zmq_strerror(zmq_errno()) << std::endl;
            continue;
        }
        rc = zmq_send(m_publisher, message.data.data(), message.data.size(), 0);
        if (rc < 0)
        {
            std::cerr << "ZmqSink: data frame send failed: " << zmq_strerror(zmq_errno()) << std::endl;
        }
#endif
    }
}
}
