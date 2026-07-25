#ifndef INCLUDED_GPSOPENCL_FILESINK_H
#define INCLUDED_GPSOPENCL_FILESINK_H

#include "GPSOpenClSink.h"
#include <fstream>
#include <mutex>
#include <string>

namespace GPSOpenCl
{
class FileSink : public Sink
{
  public:
    explicit FileSink(const std::string &outputFilePath);
    ~FileSink() override;

    void publish(const std::string &identifier, const void *data, size_t size) override;

  private:
    std::ofstream m_file;
    std::mutex m_mutex;
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_FILESINK_H
