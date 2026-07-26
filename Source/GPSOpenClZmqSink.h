#ifndef INCLUDED_GPSOPENCL_ZMQSINK_H
#define INCLUDED_GPSOPENCL_ZMQSINK_H

/** @file GPSOpenClZmqSink.h
 *  @brief ZMQ PUB socket telemetry sink.
 */

#include "GPSOpenClSink.h"
#include <mutex>
#include <string>

#ifdef GPSOPENCL_ENABLE_ZMQ
#include <zmq.h>
#endif

namespace GPSOpenCl
{
/** @brief Telemetry sink that publishes via ZMQ PUB socket. */
class ZmqSink : public Sink
{
  public:
    /** @brief Construct with ZMQ endpoint.
     *  @param endpoint ZMQ endpoint URI. */
    explicit ZmqSink(const std::string &endpoint = "ipc:///tmp/gpsopencl/telemetry.sock");
    ~ZmqSink() override;

    /** @brief Publish telemetry data over ZMQ.
     *  @param identifier Topic name.
     *  @param data       Struct data pointer.
     *  @param size       Data size (bytes). */
    void publish(const std::string &identifier, const void *data, size_t size) override;

  private:
    std::string m_endpoint;   ///< ZMQ endpoint URI.
    std::mutex m_mutex;       ///< Publish mutex.
#ifdef GPSOPENCL_ENABLE_ZMQ
    void *m_context{nullptr};   ///< ZMQ context.
    void *m_publisher{nullptr}; ///< ZMQ PUB socket.
#endif
};
}

#endif
