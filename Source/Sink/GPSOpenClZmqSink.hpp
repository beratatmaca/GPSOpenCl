#ifndef INCLUDED_GPSOPENCL_ZMQSINK_HPP
#define INCLUDED_GPSOPENCL_ZMQSINK_HPP

/** @file GPSOpenClZmqSink.hpp
 *  @brief ZMQ PUB socket telemetry sink.
 */

#include "Common/GPSOpenClBoundedQueue.hpp"
#include "Sink/GPSOpenClSink.hpp"
#include <string>
#include <thread>

#ifdef GPSOPENCL_ENABLE_ZMQ
#include <zmq.h>
#endif

namespace GPSOpenCl
{
/** @brief Default ZMQ PUB endpoint for telemetry, when none is given. */
inline const std::string ZMQ_DEFAULT_TELEMETRY_ENDPOINT = "ipc:///tmp/gpsopencl/telemetry.sock";

/** @brief Telemetry sink publishing via a ZMQ PUB socket. publish() copies onto a bounded queue
 *   and returns. A background thread does the zmq_send calls. A slow subscriber never stalls the
 *   caller. A full queue drops new messages. This matches the sink may drop design. */
class ZmqSink : public Sink
{
  public:
    /** @brief Construct with ZMQ endpoint.
     *  @param endpoint ZMQ endpoint URI. */
    explicit ZmqSink(std::string endpoint = ZMQ_DEFAULT_TELEMETRY_ENDPOINT);

    /** @brief Stop the sender thread and tear down the ZMQ socket. */
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
    /** @brief Background thread body. Drains the queue and sends each message. Stops when the
     *   queue finishes empty. */
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
