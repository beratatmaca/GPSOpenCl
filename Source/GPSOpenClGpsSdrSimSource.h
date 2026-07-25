#ifndef INCLUDED_GPSOPENCL_GPSSDRSIMSOURCE_H
#define INCLUDED_GPSOPENCL_GPSSDRSIMSOURCE_H

#include "GPSOpenClSource.h"
#include <string>

namespace GPSOpenCl
{
class GpsSdrSimSource : public Source
{
  public:
    GpsSdrSimSource();
    ~GpsSdrSimSource() override;

    bool initialize(const SourceInput &input) override;
    bool readBlock(ComplexFloatVector &outputSamples, SourceOutput &telemetry) override;
    bool sendControlCommand(const std::string &command);

  private:
    SourceInput m_inputConfig;
    std::string m_dataFifoPath;
    std::string m_ctrlFifoPath;
    int m_dataFd{-1};
    int m_ctrlFd{-1};
    uint32_t m_blockIndex{0};
    uint32_t m_underrunCount{0};
    uint32_t m_overrunCount{0};
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_GPSSDRSIMSOURCE_H
