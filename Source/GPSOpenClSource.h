#ifndef INCLUDED_GPSOPENCL_SOURCE_H
#define INCLUDED_GPSOPENCL_SOURCE_H

#include "GPSOpenClCommon.h"
#include "GPSOpenClSink.h"
#include "GPSOpenClStructs.h"

#include <memory>

namespace GPSOpenCl
{
class Source
{
  public:
    virtual ~Source() = default;

    virtual bool initialize(const SourceInput &input) = 0;
    virtual bool readBlock(ComplexFloatVector &outputSamples, SourceOutput &telemetry) = 0;
    virtual void setSink(std::shared_ptr<Sink> sink) { m_sink = sink; }

  protected:
    std::shared_ptr<Sink> m_sink{nullptr};
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_SOURCE_H
