#ifndef INCLUDED_GPSOPENCL_FILESINK_HPP
#define INCLUDED_GPSOPENCL_FILESINK_HPP

/** @file GPSOpenClFileSink.hpp
 *  @brief File-based telemetry sink.
 */

#include "Common/GPSOpenClBoundedQueue.hpp"
#include "Sink/GPSOpenClSink.hpp"
#include <fstream>
#include <string>
#include <thread>

namespace GPSOpenCl
{
/** @brief Telemetry sink writing a binary log file. publish() copies onto a bounded queue and
 *   returns. A background thread does the file I/O. A stalled disk never blocks the caller. A
 *   full queue drops new messages. This matches the sink may drop design. */
class FileSink : public Sink
{
  public:
    /** @brief Construct with output file path.
     *  @param outputFilePath Path to output file. */
    explicit FileSink(const std::string &outputFilePath);

    /** @brief Stop the writer thread and close the output file. */
    ~FileSink() override;
    FileSink(const FileSink &) = delete;
    FileSink &operator=(const FileSink &) = delete;
    FileSink(FileSink &&) = delete;
    FileSink &operator=(FileSink &&) = delete;

    /** @brief Enqueue telemetry data for the background writer thread.
     *  @param identifier Topic name.
     *  @param data       Struct data pointer.
     *  @param size       Data size (bytes). */
    void publish(const std::string &identifier, const void *data, size_t size) override;

  private:
    /** @brief Background thread body. Drains the queue and writes each message. Stops when the
     *   queue finishes empty. */
    void writerThreadLoop();

    std::ofstream m_file;                 ///< Output file stream, touched only by the writer thread.
    BoundedQueue<SinkMessage> m_queue;    ///< Handoff queue from callers to the writer thread.
    std::thread m_writerThread;           ///< Background file-writer thread.
};
}

#endif
