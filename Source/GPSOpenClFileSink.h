#ifndef INCLUDED_GPSOPENCL_FILESINK_H
#define INCLUDED_GPSOPENCL_FILESINK_H

/** @file GPSOpenClFileSink.h
 *  @brief File-based telemetry sink.
 */

#include "GPSOpenClSink.h"
#include <fstream>
#include <mutex>
#include <string>

namespace GPSOpenCl
{
/** @brief Telemetry sink that writes to a binary log file. */
class FileSink : public Sink
{
  public:
    /** @brief Construct with output file path.
     *  @param outputFilePath Path to output file. */
    explicit FileSink(const std::string &outputFilePath);
    ~FileSink() override;

    /** @brief Write telemetry data to file.
     *  @param identifier Topic name.
     *  @param data       Struct data pointer.
     *  @param size       Data size (bytes). */
    void publish(const std::string &identifier, const void *data, size_t size) override;

  private:
    std::ofstream m_file;  ///< Output file stream.
    std::mutex m_mutex;    ///< Write mutex.
};
}

#endif
