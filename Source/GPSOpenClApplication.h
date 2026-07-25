#ifndef INCLUDED_GPSOPENCL_APPLICATION_H
#define INCLUDED_GPSOPENCL_APPLICATION_H

#include "GPSOpenClAcquisition.h"
#include "GPSOpenClAtmosphericCorrections.h"
#include "GPSOpenClChannel.h"
#include "GPSOpenClCode.h"
#include "GPSOpenClGPUCompute.h"
#include "GPSOpenClNmeaGenerator.h"
#include "GPSOpenClNavigationDecoder.h"
#include "GPSOpenClPVTSolver.h"
#include "GPSOpenClProfiler.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClSink.h"
#include "GPSOpenClSource.h"
#include "GPSOpenClStructs.h"
#include "GPSOpenClTracking.h"

#include <memory>
#include <string>

namespace GPSOpenCl
{
class Application
{
  public:
    Application(Settings::Configuration conf);
    ~Application();

    void searchForSatellites(const ComplexFloatVector &input);
    void trackSatellites(const ComplexFloatVector &input);
    bool computeNavigationSolution(ReceiverPvtSolution &solution);
    void exportTelemetryJson(const std::string &filepath, const ReceiverPvtSolution &solution);

    void setSink(std::shared_ptr<Sink> sink);
    void setSource(std::shared_ptr<Source> source);
    Profiler &getProfiler() { return m_profiler; }
    void processBlock(const ComplexFloatVector &input, uint32_t blockIndex);

  private:
    void initializeChannels();

    Acquisition *m_acquisition;
    Tracking *m_tracking;
    Settings::Configuration m_configuration;

    Code *m_code;
    Compute *m_gpu;
    PVTSolver m_pvtSolver;
    NavigationDecoder m_navDecoder;
    Channel m_channels[GPS_CA_SV_COUNT];

    std::shared_ptr<Sink> m_sink{nullptr};
    std::shared_ptr<Source> m_source{nullptr};
    Profiler m_profiler;
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_APPLICATION_H
