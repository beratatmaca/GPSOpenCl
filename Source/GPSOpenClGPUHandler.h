#ifndef INCLUDED_GPSOPENCL_GPUHANDLER_H
#define INCLUDED_GPSOPENCL_GPUHANDLER_H

#include <CL/cl.h>

#include <string>
#include <vector>

namespace GPSOpenCl
{
class GpuHandler
{
  public:
    GpuHandler();
    ~GpuHandler();

    typedef enum GPUProgramList
    {
        GPSOpenClAcquistion = 0,
        GPSOpenClTracking,
        GPSOpenClProgramCount
    } GPUProgramListType;

    typedef enum GPSOpenClAcquisitionKernelList
    {
        FFTInit = 0,
        FFTStage,
        FFTScale,
        ComplexMultiplier,
        Absolute,
        Sum,
        AcquisitionKernelCount
    } GPSOpenClAcquisitionKernelListType;

    typedef enum GPSOpenClTrackingKernelList
    {
        NCOMultiplicate = 0,
        TrackingKernelCount
    } GPSOpenClTrackingKernelListListType;

    int createDevice();
    int buildProgram();
    int initKernels();
    void getLastErrorAsString();
    int determineLocalMemorySize();

    cl_context m_context;
    std::vector<cl_program> m_programList;
    std::vector<cl_kernel> m_acquisitionKernelList;
    std::vector<cl_kernel> m_trackingKernelList;
    cl_device_id m_device;
    cl_ulong m_localMemorySize;

  private:
    cl_platform_id m_platform;
    cl_int m_error;
    std::string ProgramCharList[GPSOpenClProgramCount]{"../../Kernels/Acquisition.cl", "../../Kernels/Tracking.cl"};

    std::string GPSOpenClAcquisitionKernelCharList[AcquisitionKernelCount]{
        "fft_init", "fft_stage", "fft_scale", "complexMultiplier", "absolute", "sum"};

    std::string GPSOpenClTrackingKernelCharList[TrackingKernelCount]{"ncoMultiplicate"};
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_GPUHANDLER_H