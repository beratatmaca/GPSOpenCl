#ifndef INCLUDED_GPSOPENCL_GPUHANDLER_H
#define INCLUDED_GPSOPENCL_GPUHANDLER_H

/** @file GPSOpenClGPUHandler.h
 *  @brief OpenCL device, program, and kernel manager.
 */

#include <CL/cl.h>

#include <string>
#include <vector>

namespace GPSOpenCl
{
/** @brief OpenCL device and kernel lifecycle manager. */
class GpuHandler
{
  public:
    GpuHandler();
    ~GpuHandler();

    /** @brief OpenCL program index. */
    typedef enum GPUProgramList
    {
        GPSOpenClAcquistion = 0,  ///< Acquisition kernel program.
        GPSOpenClTracking,        ///< Tracking kernel program.
        GPSOpenClProgramCount     ///< Total program count.
    } GPUProgramListType;

    /** @brief Acquisition kernel index. */
    typedef enum GPSOpenClAcquisitionKernelList
    {
        FFTInit = 0,          ///< FFT initialization kernel.
        FFTStage,             ///< FFT stage kernel.
        FFTScale,             ///< FFT IFFT scaling kernel.
        ComplexMultiplier,    ///< Complex multiplication kernel.
        Absolute,             ///< Magnitude squared kernel.
        Sum,                  ///< Reduction sum kernel.
        AcquisitionKernelCount ///< Total acquisition kernel count.
    } GPSOpenClAcquisitionKernelListType;

    /** @brief Tracking kernel index. */
    typedef enum GPSOpenClTrackingKernelList
    {
        NCOMultiplicate = 0,   ///< NCO carrier wipe kernel.
        TrackingKernelCount    ///< Total tracking kernel count.
    } GPSOpenClTrackingKernelListListType;

    /** @brief Create OpenCL device and context.
     *  @return 0 on success. */
    int createDevice();

    /** @brief Build OpenCL programs from source.
     *  @return 0 on success. */
    int buildProgram();

    /** @brief Initialize all OpenCL kernels.
     *  @return 0 on success. */
    int initKernels();

    /** @brief Log the last OpenCL error as string. */
    void getLastErrorAsString();

    /** @brief Query device local memory size.
     *  @return 0 on success. */
    int determineLocalMemorySize();

    cl_context m_context;                              ///< OpenCL context.
    std::vector<cl_program> m_programList;              ///< Compiled programs.
    std::vector<cl_kernel> m_acquisitionKernelList;     ///< Acquisition kernels.
    std::vector<cl_kernel> m_trackingKernelList;        ///< Tracking kernels.
    cl_device_id m_device;                              ///< OpenCL device ID.
    cl_ulong m_localMemorySize;                         ///< Device local memory (bytes).

  private:
    cl_platform_id m_platform;                          ///< OpenCL platform ID.
    cl_int m_error;                                     ///< Last OpenCL error code.
    std::string ProgramCharList[GPSOpenClProgramCount]{"Acquisition.cl", "Tracking.cl"};

    std::string GPSOpenClAcquisitionKernelCharList[AcquisitionKernelCount]{
        "fft_init", "fft_stage", "fft_scale", "complexMultiplier", "absolute", "sum"};

    std::string GPSOpenClTrackingKernelCharList[TrackingKernelCount]{"ncoMultiplicate"};
};
}

#endif