#ifndef INCLUDED_GPSOPENCL_SOURCE_H
#define INCLUDED_GPSOPENCL_SOURCE_H

/** @file GPSOpenClSource.h
 *  @brief Abstract IQ sample source interface.
 */

#include "GPSOpenClCommon.h"
#include "GPSOpenClSink.h"
#include "GPSOpenClStructs.h"

#include <memory>
#include <utility>

namespace GPSOpenCl
{
/** @brief Abstract baseband sample source.
 *  @see FileSource, GpsSdrSimSource */
class Source
{
  public:
    virtual ~Source() = default;
    Source() = default;
    Source(const Source &) = delete;
    Source &operator=(const Source &) = delete;
    Source(Source &&) = delete;
    Source &operator=(Source &&) = delete;

    /** @brief Initialize the source.
     *  @param input Source configuration.
     *  @return True if initialized. */
    virtual bool initialize(const SourceInput &input) = 0;

    /** @brief Read one block of IQ samples.
     *  @param outputSamples Output sample buffer.
     *  @param telemetry     Output telemetry.
     *  @return True if block read. */
    virtual bool readBlock(ComplexFloatVector &outputSamples, SourceOutput &telemetry) = 0;

    /** @brief Set telemetry sink.
     *  @param sink Sink implementation. */
    virtual void setSink(std::shared_ptr<Sink> sink) { m_sink = std::move(sink); }

  protected:
    std::shared_ptr<Sink> m_sink{nullptr};    ///< Telemetry sink.
};
}

#endif
