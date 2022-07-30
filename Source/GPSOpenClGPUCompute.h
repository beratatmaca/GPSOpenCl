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

    int fft(const ComplexFloatVector input, ComplexFloatVector *output, FFTDirectionType direction);
    int complexMultiplier(const ComplexFloatVector input1, const ComplexFloatVector input2, ComplexFloatVector *output);
    int absolute(const ComplexFloatVector input1, FloatVector *output);
    int sum(const FloatVector input, float *sumValue);
    int ncoMultiplication(const ComplexFloatVector input, const FloatVector phaseVector, ComplexFloatVector *output);

  private:
    GpuHandler m_gpu;
    cl_command_queue m_queue;
    cl_int m_error;
    float m_allocatedMemory[DEFAULT_MAX_ALLOCATION];
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_COMPUTE_H