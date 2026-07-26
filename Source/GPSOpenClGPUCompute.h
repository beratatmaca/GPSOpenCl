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
    /** @brief Query and cache per-kernel work-group size and device local memory size, once. */
    void cacheDeviceInfo();

    /** @brief Return a persistent device buffer sized for at least neededFloats floats, growing
     *   (release + recreate) only when the current capacity is insufficient.
     *  @param buffer         Persistent buffer handle (in/out).
     *  @param capacityFloats Current buffer capacity in floats (in/out).
     *  @param neededFloats   Required capacity in floats.
     *  @return Buffer handle, or nullptr on allocation failure. */
    cl_mem ensureBuffer(cl_mem &buffer, size_t &capacityFloats, size_t neededFloats);

    GpuHandler m_gpu;                      ///< OpenCL handler.
    cl_command_queue m_queue;               ///< OpenCL command queue.
    cl_int m_error;                         ///< Last OpenCL error.
    std::vector<float> m_allocatedMemory;   ///< Scratch memory buffer.
    std::vector<float> m_partialSums;       ///< Scratch buffer for sum() partial-sum readback.

    bool m_deviceInfoCached{false};              ///< True once per-kernel/device info below is cached.
    cl_ulong m_localMemorySize{0};               ///< Cached device local memory size (bytes).
    size_t m_fftLocalSize{0};                     ///< Cached fft_init kernel work-group size.
    size_t m_complexMultiplierLocalSize{0};       ///< Cached complexMultiplier kernel work-group size.
    size_t m_absoluteLocalSize{0};                ///< Cached absolute kernel work-group size.
    size_t m_sumLocalSize{0};                     ///< Cached sum kernel work-group size.
    size_t m_ncoLocalSize{0};                     ///< Cached ncoMultiplicate kernel work-group size.

    cl_mem m_fftBuffer{nullptr};                  ///< Persistent FFT data buffer.
    size_t m_fftBufferCapacity{0};                ///< FFT buffer capacity (floats).

    cl_mem m_cmBufferA{nullptr};                  ///< Persistent complexMultiplier input-A buffer.
    cl_mem m_cmBufferB{nullptr};                  ///< Persistent complexMultiplier input-B buffer.
    cl_mem m_cmBufferC{nullptr};                  ///< Persistent complexMultiplier output buffer.
    size_t m_cmBufferCapacityA{0};                 ///< complexMultiplier input-A buffer capacity (floats).
    size_t m_cmBufferCapacityB{0};                 ///< complexMultiplier input-B buffer capacity (floats).
    size_t m_cmBufferCapacityC{0};                 ///< complexMultiplier output buffer capacity (floats).

    cl_mem m_absBufferA{nullptr};                  ///< Persistent absolute() input buffer.
    cl_mem m_absBufferC{nullptr};                  ///< Persistent absolute() output buffer.
    size_t m_absBufferCapacityA{0};                ///< absolute() input buffer capacity (floats).
    size_t m_absBufferCapacityC{0};                ///< absolute() output buffer capacity (floats).

    cl_mem m_sumBufferInput{nullptr};              ///< Persistent sum() input buffer.
    cl_mem m_sumBufferOutput{nullptr};              ///< Persistent sum() partial-sum output buffer.
    size_t m_sumBufferInputCapacity{0};            ///< sum() input buffer capacity (floats).
    size_t m_sumBufferOutputCapacity{0};           ///< sum() output buffer capacity (floats, one per work-group).

    cl_mem m_ncoBufferData{nullptr};               ///< Persistent ncoMultiplication() IQ data buffer.
    cl_mem m_ncoBufferPhase{nullptr};              ///< Persistent ncoMultiplication() phase buffer.
    size_t m_ncoBufferDataCapacity{0};             ///< ncoMultiplication() data buffer capacity (floats).
    size_t m_ncoBufferPhaseCapacity{0};            ///< ncoMultiplication() phase buffer capacity (floats).
};
}

#endif