#ifndef INCLUDED_GPSOPENCL_FILESINK_H
#define INCLUDED_GPSOPENCL_FILESINK_H

/** @file GPSOpenClFileSink.h
 *  @brief File-based telemetry sink.
 */

#include "GPSOpenClBoundedQueue.h"
#include "GPSOpenClSink.h"
#include <fstream>
#include <string>
#include <thread>

namespace GPSOpenCl
{
/** @brief Telemetry sink that writes to a binary log file. publish() copies onto a bounded queue
 *   and returns immediately; a dedicated background thread does the actual file I/O, so a slow or
 *   stalled disk never blocks the calling (real-time) thread. The queue drops new messages once
 *   full rather than blocking the caller, matching the project's "Sink path may drop" design. */
class FileSink : public Sink
{
  public:
    /** @brief Construct with output file path.
     *  @param outputFilePath Path to output file. */
    explicit FileSink(const std::string &outputFilePath);
    ~FileSink() override;

    /** @brief Enqueue telemetry data for the background writer thread.
     *  @param identifier Topic name.
     *  @param data       Struct data pointer.
     *  @param size       Data size (bytes). */
    void publish(const std::string &identifier, const void *data, size_t size) override;

  private:
    /** @brief Background thread body: drains the queue and writes each message to file until
     *   the queue is finished and empty. */
    void writerThreadLoop();

    std::ofstream m_file;                  ///< Output file stream, touched only by the writer thread.
    BoundedQueue<SinkMessage> m_queue;      ///< Handoff queue from callers to the writer thread.
    std::thread m_writerThread;             ///< Background file-writer thread.
};
}

#endif
