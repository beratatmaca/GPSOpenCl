#ifndef INCLUDED_GPSOPENCL_SINK_H
#define INCLUDED_GPSOPENCL_SINK_H

#include <cstddef>
#include <string>
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

} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_SINK_H
