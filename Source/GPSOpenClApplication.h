#ifndef INCLUDED_GPSOPENCL_APPLICATION_H
#define INCLUDED_GPSOPENCL_APPLICATION_H

#include "GPSOpenClAcquisition.h"
#include "GPSOpenClChannel.h"
#include "GPSOpenClCode.h"
#include "GPSOpenClGPUCompute.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClTracking.h"

namespace GPSOpenCl
{
class Application
{
  public:
    Application(Settings::Configuration conf);
    ~Application();

    void searchForSatellites(const ComplexFloatVector input);
    void trackSatellites(const ComplexFloatVector input);

  private:
    void initializeChannels();

    Acquisition *m_acquisition;
    Tracking *m_tracking;
    Settings::Configuration m_configuration;

    Code *m_code;
    Compute *m_gpu;
    Channel m_channels[GPS_CA_SV_COUNT];
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_ACQUISITION_H
