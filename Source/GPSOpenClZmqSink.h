#ifndef INCLUDED_GPSOPENCL_ZMQSINK_H
#define INCLUDED_GPSOPENCL_ZMQSINK_H

#include "GPSOpenClSink.h"
#include <mutex>
#include <string>

#ifdef GPSOPENCL_ENABLE_ZMQ
#include <zmq.h>
#endif

namespace GPSOpenCl
{
class ZmqSink : public Sink
{
  public:
    explicit ZmqSink(const std::string &endpoint = "ipc:///tmp/gpsopencl/telemetry.sock");
    ~ZmqSink() override;

    void publish(const std::string &identifier, const void *data, size_t size) override;

  private:
    std::string m_endpoint;
    std::mutex m_mutex;
#ifdef GPSOPENCL_ENABLE_ZMQ
    void *m_context{nullptr};
    void *m_publisher{nullptr};
#endif
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_ZMQSINK_H
