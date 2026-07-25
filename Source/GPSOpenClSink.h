#ifndef INCLUDED_GPSOPENCL_SINK_H
#define INCLUDED_GPSOPENCL_SINK_H

#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include "GPSOpenClStructs.h"

namespace GPSOpenCl
{
class Sink
{
  public:
    virtual ~Sink() = default;

    virtual void publish(const std::string &identifier, const void *data, size_t size) = 0;

    void publishSourceOutput(const SourceOutput &out)
    {
        publish("SourceOutput", &out, sizeof(out));
    }

    void publishAcquisitionOutput(const AcquisitionOutput &out)
    {
        publish("AcquisitionOutput", &out, sizeof(out));
    }

    void publishTrackingOutput(const TrackingOutput &out)
    {
        publish("TrackingOutput", &out, sizeof(out));
    }

    void publishNavDecoderOutput(const NavDecoderOutput &out)
    {
        publish("NavDecoderOutput", &out, sizeof(out));
    }

    void publishPvtSolverOutput(const PvtSolverOutput &out)
    {
        publish("PvtSolverOutput", &out, sizeof(out));
    }

    void publishAtmosphericOutput(const AtmosphericOutput &out)
    {
        publish("AtmosphericOutput", &out, sizeof(out));
    }

    void publishNmeaGeneratorOutput(const NmeaGeneratorOutput &out)
    {
        publish("NmeaGeneratorOutput", &out, sizeof(out));
    }

    void publishProfilerOutput(const ProfilerOutput &out)
    {
        publish("ProfilerOutput", &out, sizeof(out));
    }
};

class NullSink : public Sink
{
  public:
    void publish(const std::string &/*identifier*/, const void */*data*/, size_t /*size*/) override
    {
        // No-op
    }
};

class CompositeSink : public Sink
{
  public:
    void addSink(std::shared_ptr<Sink> sink)
    {
        if (sink) m_sinks.push_back(sink);
    }

    void publish(const std::string &identifier, const void *data, size_t size) override
    {
        for (auto &s : m_sinks)
        {
            s->publish(identifier, data, size);
        }
    }

  private:
    std::vector<std::shared_ptr<Sink>> m_sinks;
};

} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_SINK_H
