#ifndef INCLUDED_GPSOPENCL_GPUHANDLER_H
#define INCLUDED_GPSOPENCL_GPUHANDLER_H

/** @file GPSOpenClGPUHandler.h
 *  @brief OpenCL device, program, and kernel manager.
 */

#include <CL/cl.h>

#include <cstdint>
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
    GpuHandler(const GpuHandler &) = delete;
    GpuHandler &operator=(const GpuHandler &) = delete;
    GpuHandler(GpuHandler &&) = delete;
    GpuHandler &operator=(GpuHandler &&) = delete;

    /** @brief OpenCL program index. */
    using GPUProgramListType = enum GPUProgramList : std::uint8_t
    {
        GPSOpenClAcquistion = 0,    ///< Acquisition kernel program.
        GPSOpenClProgramCount       ///< Total program count.
    };

    /** @brief Acquisition kernel index. */
    using GPSOpenClAcquisitionKernelListType = enum GPSOpenClAcquisitionKernelList : std::uint8_t
    {
        FFTInit = 0,               ///< FFT initialization kernel.
        FFTStage,                  ///< FFT stage kernel.
        FFTScale,                  ///< FFT IFFT scaling kernel.
        ComplexMultiplier,         ///< Complex multiplication kernel.
        Absolute,                  ///< Magnitude squared kernel.
        ComplexMultiplierBatch,    ///< Batched per-Doppler-bin complex multiplication kernel.
        AcquisitionKernelCount     ///< Total acquisition kernel count.
    };

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
    void getLastErrorAsString() const;

    /** @brief Query device local memory size.
     *  @return 0 on success. */
    int determineLocalMemorySize();

    cl_context context{nullptr};                     ///< OpenCL context.
    std::vector<cl_program> programList;             ///< Compiled programs.
    std::vector<cl_kernel> acquisitionKernelList;    ///< Acquisition kernels.
    cl_device_id device{nullptr};                    ///< OpenCL device ID.
    cl_ulong localMemorySize{0};                     ///< Device local memory (bytes).

  private:
    cl_platform_id m_platform{nullptr};    ///< OpenCL platform ID.
    cl_int m_lastError{0};                 ///< Last OpenCL error code.
    std::string m_programCharList[GPSOpenClProgramCount]{"Acquisition.cl"};

    std::string m_acquisitionKernelCharList[AcquisitionKernelCount]{"fft_init",
                                                                    "fft_stage",
                                                                    "fft_scale",
                                                                    "complexMultiplier",
                                                                    "absolute",
                                                                    "complexMultiplierBatch"};
};
}

#endif
