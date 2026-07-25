#ifndef INCLUDED_GPSOPENCL_COMPUTE_H
#define INCLUDED_GPSOPENCL_COMPUTE_H

/** @file GPSOpenClGPUCompute.h
 *  @brief FFT, complex math, and NCO compute back-end.
 */

#include "GPSOpenClCommon.h"
#include "GPSOpenClGPUHandler.h"

namespace GPSOpenCl
{
/** @brief GPU/CPU compute engine for FFT and signal math. */
class Compute
{
  public:
    Compute();
    ~Compute();

    /** @brief FFT direction flag. */
    typedef enum FFTDirection
    {
        FFTForward = 1,  ///< Forward FFT.
        FFTInverse = -1  ///< Inverse FFT.
    } FFTDirectionType;

    /** @brief Compute FFT or IFFT.
     *  @param input     Complex input samples.
     *  @param output    Complex output samples.
     *  @param direction Forward or inverse.
     *  @return 0 on success. */
    int fft(const ComplexFloatVector &input, ComplexFloatVector *output, FFTDirectionType direction);

    /** @brief Element-wise complex multiplication.
     *  @param input1 First complex vector.
     *  @param input2 Second complex vector.
     *  @param output Product vector.
     *  @return 0 on success. */
    int complexMultiplier(const ComplexFloatVector &input1, const ComplexFloatVector &input2, ComplexFloatVector *output);

    /** @brief Compute magnitude squared of complex vector.
     *  @param input1 Complex input.
     *  @param output Magnitude squared output.
     *  @return 0 on success. */
    int absolute(const ComplexFloatVector &input1, FloatVector *output);

    /** @brief Parallel reduction sum.
     *  @param input    Float vector.
     *  @param sumValue Output sum.
     *  @return 0 on success. */
    int sum(const FloatVector &input, float *sumValue);

    /** @brief NCO carrier multiplication.
     *  @param input       Complex IQ samples.
     *  @param phaseVector NCO phase ramp (rad).
     *  @param output      Carrier-wiped output.
     *  @return 0 on success. */
    int ncoMultiplication(const ComplexFloatVector &input, const FloatVector &phaseVector, ComplexFloatVector *output);

    /** @brief Round down to the nearest power of two.
     *  @param value Input value (0 stays 0).
     *  @return Nearest power of two not greater than value. */
    static unsigned int roundDownToPowerOfTwo(unsigned int value);

    /** @brief Shrink a work-group size until pointsPerGroup/localSize meets a minimum.
     *  @param localSize        Work-group size, assumed already a power of two.
     *  @param pointsPerGroup   Points processed per work-group.
     *  @param minPointsPerItem Minimum points each work-item must process.
     *  @return Clamped work-group size. */
    static unsigned int clampLocalSizeForMinPointsPerItem(unsigned int localSize, unsigned int pointsPerGroup,
                                                          unsigned int minPointsPerItem);

  private:
    GpuHandler m_gpu;                      ///< OpenCL handler.
    cl_command_queue m_queue;               ///< OpenCL command queue.
    cl_int m_error;                         ///< Last OpenCL error.
    std::vector<float> m_allocatedMemory;   ///< Scratch memory buffer.
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_COMPUTE_H