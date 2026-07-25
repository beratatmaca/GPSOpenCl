#include "GPSOpenClZmqSink.h"
#include <iostream>

namespace GPSOpenCl
{
ZmqSink::ZmqSink(const std::string &endpoint) : m_endpoint(endpoint)
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
}

ZmqSink::~ZmqSink()
{
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
    std::lock_guard<std::mutex> lock(m_mutex);
#ifdef GPSOPENCL_ENABLE_ZMQ
    if (!m_publisher) return;

    zmq_send(m_publisher, identifier.data(), identifier.size(), ZMQ_SNDMORE);
    zmq_send(m_publisher, data, size, 0);
#else
    (void)identifier;
    (void)data;
    (void)size;
#endif
}
}
