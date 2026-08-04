#ifndef INCLUDED_GPSOPENCL_ZMQSINK_H
#define INCLUDED_GPSOPENCL_ZMQSINK_H

/** @file GPSOpenClZmqSink.h
 *  @brief ZMQ PUB socket telemetry sink.
 */

#include "GPSOpenClBoundedQueue.h"
#include "GPSOpenClSink.h"
#include <string>
#include <thread>

#ifdef GPSOPENCL_ENABLE_ZMQ
#include <zmq.h>
#endif

namespace GPSOpenCl
{
/** @brief Telemetry sink that publishes via ZMQ PUB socket. publish() copies onto a bounded queue
 *   and returns immediately; a dedicated background thread does the actual zmq_send calls, so a
 *   blocked or slow subscriber never stalls the calling (real-time) thread. The queue drops new
 *   messages once full rather than blocking the caller, matching the project's "Sink path may
 *   drop" design. */
class ZmqSink : public Sink
{
  public:
    /** @brief Construct with ZMQ endpoint.
     *  @param endpoint ZMQ endpoint URI. */
    explicit ZmqSink(std::string endpoint = "ipc:///tmp/gpsopencl/telemetry.sock");
    ~ZmqSink() override;
    ZmqSink(const ZmqSink &) = delete;
    ZmqSink &operator=(const ZmqSink &) = delete;
    ZmqSink(ZmqSink &&) = delete;
    ZmqSink &operator=(ZmqSink &&) = delete;

    /** @brief Enqueue telemetry data for the background sender thread.
     *  @param identifier Topic name.
     *  @param data       Struct data pointer.
     *  @param size       Data size (bytes). */
    void publish(const std::string &identifier, const void *data, size_t size) override;

  private:
    /** @brief Background thread body: drains the queue and sends each message over ZMQ until
     *   the queue is finished and empty. */
    void senderThreadLoop();

    std::string m_endpoint;               ///< ZMQ endpoint URI.
    BoundedQueue<SinkMessage> m_queue;    ///< Handoff queue from callers to the sender thread.
    std::thread m_senderThread;           ///< Background ZMQ-sender thread.
#ifdef GPSOPENCL_ENABLE_ZMQ
    void *m_context{nullptr};             ///< ZMQ context.
    void *m_publisher{nullptr};           ///< ZMQ PUB socket.
#endif
};
}

#endif
