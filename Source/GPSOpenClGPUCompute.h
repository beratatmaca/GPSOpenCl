#ifndef INCLUDED_GPSOPENCL_COMPUTE_H
#define INCLUDED_GPSOPENCL_COMPUTE_H

#include "GPSOpenClCommon.h"
#include "GPSOpenClGPUHandler.h"

namespace GPSOpenCl
{
class Compute
{
  public:
    Compute();
    ~Compute();

    typedef enum FFTDirection
    {
        FFTForward = 1,
        FFTInverse = -1
    } FFTDirectionType;

    int fft(const ComplexFloatVector &input, ComplexFloatVector *output, FFTDirectionType direction);
    int complexMultiplier(const ComplexFloatVector &input1, const ComplexFloatVector &input2, ComplexFloatVector *output);
    int absolute(const ComplexFloatVector &input1, FloatVector *output);
    int sum(const FloatVector &input, float *sumValue);
    int ncoMultiplication(const ComplexFloatVector &input, const FloatVector &phaseVector, ComplexFloatVector *output);

    // Rounds down to the nearest power of two (0 stays 0), so a work-group/points-per-group size
    // evenly divides any power-of-two buffer length instead of silently truncating.
    static unsigned int roundDownToPowerOfTwo(unsigned int value);

    // Shrinks localSize (assumed already a power of two) until pointsPerGroup/localSize is at
    // least minPointsPerItem, so a kernel's points_per_item = points_per_group/get_local_size(0)
    // never truncates to less than the kernel's own minimum chunk size (e.g. 0, which hangs a
    // kernel looping "while (N2 < points_per_group) N2 <<= 1" from a stuck N2=0; or 1-3 when the
    // kernel processes points in fixed groups of 4, which silently corrupts results). See
    // Kernels/Acquisition.cl's fft_init/fft_stage and Kernels/Tracking.cl's ncoMultiplicate.
    static unsigned int clampLocalSizeForMinPointsPerItem(unsigned int localSize, unsigned int pointsPerGroup,
                                                          unsigned int minPointsPerItem);

  private:
    GpuHandler m_gpu;
    cl_command_queue m_queue;
    cl_int m_error;
    std::vector<float> m_allocatedMemory;
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_COMPUTE_H