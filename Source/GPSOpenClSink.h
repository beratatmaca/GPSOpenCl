#ifndef INCLUDED_GPSOPENCL_SINK_H
#define INCLUDED_GPSOPENCL_SINK_H

/** @file GPSOpenClSink.h
 *  @brief Abstract telemetry output interface and implementations.
 */

#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include "GPSOpenClStructs.h"

namespace GPSOpenCl
{
/** @brief Abstract telemetry output interface. */
class Sink
{
  public:
    virtual ~Sink() = default;

    /** @brief Publish binary data with an identifier.
     *  @param identifier Topic/module name.
     *  @param data       Pointer to struct data.
     *  @param size       Size of data (bytes). */
    virtual void publish(const std::string &identifier, const void *data, size_t size) = 0;

    /** @brief Publish SourceOutput telemetry.
     *  @param out Source output struct. */
    void publishSourceOutput(const SourceOutput &out)
    {
        publish("SourceOutput", &out, sizeof(out));
    }

    /** @brief Publish AcquisitionOutput telemetry.
     *  @param out Acquisition output struct. */
    void publishAcquisitionOutput(const AcquisitionOutput &out)
    {
        publish("AcquisitionOutput", &out, sizeof(out));
    }

    /** @brief Publish TrackingOutput telemetry.
     *  @param out Tracking output struct. */
    void publishTrackingOutput(const TrackingOutput &out)
    {
        publish("TrackingOutput", &out, sizeof(out));
    }

    /** @brief Publish NavDecoderOutput telemetry.
     *  @param out Nav decoder output struct. */
    void publishNavDecoderOutput(const NavDecoderOutput &out)
    {
        publish("NavDecoderOutput", &out, sizeof(out));
    }

    /** @brief Publish PvtSolverOutput telemetry.
     *  @param out PVT solver output struct. */
    void publishPvtSolverOutput(const PvtSolverOutput &out)
    {
        publish("PvtSolverOutput", &out, sizeof(out));
    }

    /** @brief Publish AtmosphericOutput telemetry.
     *  @param out Atmospheric output struct. */
    void publishAtmosphericOutput(const AtmosphericOutput &out)
    {
        publish("AtmosphericOutput", &out, sizeof(out));
    }

    /** @brief Publish NmeaGeneratorOutput telemetry.
     *  @param out NMEA output struct. */
    void publishNmeaGeneratorOutput(const NmeaGeneratorOutput &out)
    {
        publish("NmeaGeneratorOutput", &out, sizeof(out));
    }

    /** @brief Publish ProfilerOutput telemetry.
     *  @param out Profiler output struct. */
    void publishProfilerOutput(const ProfilerOutput &out)
    {
        publish("ProfilerOutput", &out, sizeof(out));
    }
};

/** @brief No-op sink that discards all telemetry. */
class NullSink : public Sink
{
  public:
    /** @brief Discard published data. */
    void publish(const std::string &, const void *, size_t) override
    {
    }
};

/** @brief Fan-out sink that forwards to multiple downstream sinks. */
class CompositeSink : public Sink
{
  public:
    /** @brief Add a downstream sink.
     *  @param sink Sink to add. */
    void addSink(std::shared_ptr<Sink> sink)
    {
        if (sink) m_sinks.push_back(sink);
    }

    /** @brief Forward data to all downstream sinks. */
    void publish(const std::string &identifier, const void *data, size_t size) override
    {
        for (auto &s : m_sinks)
        {
            s->publish(identifier, data, size);
        }
    }

  private:
    std::vector<std::shared_ptr<Sink>> m_sinks; ///< Downstream sinks.
};

} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_SINK_H
