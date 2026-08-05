#ifndef INCLUDED_GPSOPENCL_COMPUTE_H
#define INCLUDED_GPSOPENCL_COMPUTE_H

/** @file GPSOpenClGPUCompute.h
 *  @brief FFT, complex math, and NCO compute back-end.
 */

#include "GPSOpenClCommon.h"
#include "GPSOpenClGPUHandler.h"

#include <cstdint>
#include <utility>
#include <vector>

namespace GPSOpenCl
{
/** @brief GPU/CPU compute engine for FFT and signal math. */
class Compute
{
  public:
    Compute();
    ~Compute();
    Compute(const Compute &) = delete;
    Compute &operator=(const Compute &) = delete;
    Compute(Compute &&) = delete;
    Compute &operator=(Compute &&) = delete;

    /** @brief FFT direction flag. */
    using FFTDirectionType = enum FFTDirection : std::int8_t
    {
        FFTForward = 1,    ///< Forward FFT.
        FFTInverse = -1    ///< Inverse FFT.
    };

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
    int complexMultiplier(const ComplexFloatVector &input1,
                          const ComplexFloatVector &input2,
                          ComplexFloatVector *output);

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

    /** @brief Complex-multiply then FFT, leaving the result resident in a numbered slot (device
     *   buffer on the GPU path, host vector on the CPU fallback) instead of returning it. Pair with
     *   complexMultiplyResidentThenFftThenAbsolute() to reuse the result across many calls with no
     *   host round-trip.
     *  @param input1    First complex vector.
     *  @param input2    Second complex vector.
     *  @param direction Forward or inverse FFT.
     *  @param slot      Slot index to store the result in (grown on demand).
     *  @return 0 on success. */
    int complexMultiplyThenFftToSlot(const ComplexFloatVector &input1,
                                     const ComplexFloatVector &input2,
                                     FFTDirectionType direction,
                                     int slot);

    /** @brief Complex-multiply input1 with a circularly shifted slot-resident vector, FFT, then
     *   magnitude-squared: output = |FFT(input1[i] * slot[(i + length - shiftBins) mod length])|^2.
     *   The shift is applied through indexing during the multiply, so no shifted copy is created.
     *   input1 is cached on-device between calls and re-uploaded only when a different vector (by
     *   address, size, or slot generation) is supplied, so repeated calls with the same spectrum are
     *   upload-free. The caller must keep input1's storage unchanged between calls.
     *  @param input1    First complex vector (e.g. a precomputed constant code spectrum).
     *  @param slot      Slot index previously filled by complexMultiplyThenFftToSlot().
     *  @param shiftBins Circular shift applied to the slot vector, in [0, length).
     *  @param direction Forward or inverse FFT.
     *  @param output    Magnitude squared of the FFT of the product.
     *  @return 0 on success. */
    int complexMultiplyResidentThenFftThenAbsolute(const ComplexFloatVector &input1,
                                                   int slot,
                                                   int shiftBins,
                                                   FFTDirectionType direction,
                                                   FloatVector *output);

    /** @brief Batched form of complexMultiplyResidentThenFftThenAbsolute(): performs the multiply,
     *   FFT, and magnitude-squared for every requested (slot, shift) bin in one GPU submission with
     *   a single readback at the end, instead of one blocking round-trip per bin. Bin k's result
     *   occupies output[k * length, (k + 1) * length). Requires every requested slot to be resident
     *   with the same length as input1; returns nonzero without touching output when the batch
     *   cannot run (caller should fall back to per-bin calls).
     *  @param input1       First complex vector (e.g. a precomputed constant code spectrum).
     *  @param slotAndShift Per-bin (slot index, circular shift) pairs.
     *  @param direction    Forward or inverse FFT.
     *  @param output       Concatenated magnitude-squared results, bins * length values.
     *  @return 0 on success. */
    int complexMultiplyResidentThenFftThenAbsoluteBatch(const ComplexFloatVector &input1,
                                                        const std::vector<std::pair<int, int>> &slotAndShift,
                                                        FFTDirectionType direction,
                                                        FloatVector *output);

    /** @brief Round down to the nearest power of two.
     *  @param value Input value (0 stays 0).
     *  @return Nearest power of two not greater than value. */
    static unsigned int roundDownToPowerOfTwo(unsigned int value);

    /** @brief Clamp a candidate points-per-group value to a power of two no larger than length.
     *  @param pointsPerGroup Candidate points processed per work-group.
     *  @param length         Total points to process.
     *  @return Clamped points-per-group value. */
    static unsigned int clampPointsPerGroup(unsigned int pointsPerGroup, unsigned int length);

    /** @brief Shrink a work-group size until pointsPerGroup/localSize meets a minimum.
     *  @param localSize        Work-group size, assumed already a power of two.
     *  @param pointsPerGroup   Points processed per work-group.
     *  @param minPointsPerItem Minimum points each work-item must process.
     *  @return Clamped work-group size. */
    static unsigned int clampLocalSizeForMinPointsPerItem(unsigned int localSize,
                                                          unsigned int pointsPerGroup,
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

    /** @brief GPU complexMultiplier stage, leaving the product on-device instead of reading it
     *   back to host. Used both by complexMultiplier() and by the on-device chained calls.
     *  @param input1 First complex vector.
     *  @param input2 Second complex vector.
     *  @param length Element count (equal for both inputs).
     *  @return The device output buffer (m_cmBufferC), or nullptr on failure. */
    cl_mem complexMultiplierDevice(const ComplexFloatVector &input1,
                                   const ComplexFloatVector &input2,
                                   unsigned int length);

    /** @brief GPU complexMultiplier stage against an already-device-resident second operand, read
     *   with a circular offset. input1 is uploaded only when it differs from the previous call's
     *   cached vector (by address or size).
     *  @param input1 First complex vector (cached on-device between calls).
     *  @param input2 Device buffer holding length interleaved complex floats.
     *  @param length Element count (equal for both inputs).
     *  @param offset Circular read offset into input2, in [0, length).
     *  @return The device output buffer (m_cmBufferC), or nullptr on failure. */
    cl_mem complexMultiplierResidentDevice(const ComplexFloatVector &input1,
                                           cl_mem input2,
                                           unsigned int length,
                                           unsigned int offset);

    /** @brief Enqueue the complexMultiplier kernel on already-populated device buffers.
     *  @param inputA     First operand device buffer.
     *  @param inputB     Second operand device buffer, read with the circular offset.
     *  @param length     Element count.
     *  @param offset     Circular read offset into inputB, in [0, length).
     *  @param inputBBase Element offset of the operand vector inside inputB.
     *  @return The device output buffer (m_cmBufferC), or nullptr on failure. */
    cl_mem enqueueComplexMultiplier(cl_mem inputA,
                                    cl_mem inputB,
                                    unsigned int length,
                                    unsigned int offset,
                                    unsigned int inputBBase);

    /** @brief Upload input1 into the persistent multiply input buffer unless the same vector (by
     *   address and size) is already resident from a previous call.
     *  @param input1 Complex vector to make device-resident.
     *  @param length Element count.
     *  @return The input device buffer (m_cmBufferA), or nullptr on failure. */
    cl_mem ensureResidentInput1(const ComplexFloatVector &input1, unsigned int length);

    /** @brief GPU FFT stage over a pool of independent equal-length FFTs stored back to back in one
     *   device buffer, one work-group per FFT in a single launch. Only valid when a whole FFT fits
     *   in one work-group's local memory.
     *  @param buffer    Device buffer holding count * length interleaved complex floats (in/out).
     *  @param length    Element count of one FFT.
     *  @param count     Number of independent FFTs.
     *  @param direction Forward or inverse FFT.
     *  @return 0 on success. */
    int fftDeviceInPlaceBatch(cl_mem buffer, unsigned int length, unsigned int count, FFTDirectionType direction);

    /** @brief Pack a complex vector into interleaved-float staging memory at a float offset.
     *  @param input       Complex input vector.
     *  @param length      Element count to pack.
     *  @param floatOffset Destination offset into m_allocatedMemory (floats). */
    void packToStaging(const ComplexFloatVector &input, unsigned int length, size_t floatOffset);

    /** @brief Ensure the pooled slot device buffer can hold the given slot at the given per-slot
     *   float stride. Slots live back to back in one buffer so batched kernels can address them by
     *   element offset. Growing the pool discards previous contents, so every slot state is
     *   invalidated when that happens.
     *  @param slot          Slot index.
     *  @param floatsPerSlot Per-slot capacity in floats.
     *  @return Pool device buffer, or nullptr on allocation failure. */
    cl_mem ensureSlotPool(int slot, size_t floatsPerSlot);

    /** @brief GPU FFT stage, operating in place on a caller-supplied device buffer instead of
     *   uploading from or reading back to host. Used both by fft() and by the on-device chained
     *   calls.
     *  @param buffer    Device buffer holding length interleaved complex floats (in/out).
     *  @param length    Element count.
     *  @param direction Forward or inverse FFT.
     *  @return 0 on success. */
    int fftDeviceInPlace(cl_mem buffer, unsigned int length, FFTDirectionType direction);

    /** @brief GPU absolute stage, reading its input directly from a caller-supplied device buffer
     *   instead of uploading from host. Used both by absolute() and by the on-device chained calls.
     *  @param inputBuffer Device buffer holding length interleaved complex floats.
     *  @param length      Element count.
     *  @param output      Magnitude squared output (host).
     *  @return 0 on success. */
    int absoluteDeviceToHost(cl_mem inputBuffer, unsigned int length, FloatVector *output);

    GpuHandler m_gpu;                          ///< OpenCL handler.
    cl_command_queue m_queue;                  ///< OpenCL command queue.
    cl_int m_error;                            ///< Last OpenCL error.
    std::vector<float> m_allocatedMemory;      ///< Scratch memory buffer.
    std::vector<float> m_partialSums;          ///< Scratch buffer for sum() partial-sum readback.

    bool m_deviceInfoCached{false};            ///< True once per-kernel/device info below is cached.
    cl_ulong m_localMemorySize{0};             ///< Cached device local memory size (bytes).
    size_t m_fftLocalSize{0};                  ///< Cached fft_init kernel work-group size.
    size_t m_complexMultiplierLocalSize{0};    ///< Cached complexMultiplier kernel work-group size.
    size_t m_absoluteLocalSize{0};             ///< Cached absolute kernel work-group size.
    size_t m_sumLocalSize{0};                  ///< Cached sum kernel work-group size.

    cl_mem m_fftBuffer{nullptr};               ///< Persistent FFT data buffer.
    size_t m_fftBufferCapacity{0};             ///< FFT buffer capacity (floats).

    cl_mem m_cmBufferA{nullptr};               ///< Persistent complexMultiplier input-A buffer.
    cl_mem m_cmBufferB{nullptr};               ///< Persistent complexMultiplier input-B buffer.
    cl_mem m_cmBufferC{nullptr};               ///< Persistent complexMultiplier output buffer.
    size_t m_cmBufferCapacityA{0};             ///< complexMultiplier input-A buffer capacity (floats).
    size_t m_cmBufferCapacityB{0};             ///< complexMultiplier input-B buffer capacity (floats).
    size_t m_cmBufferCapacityC{0};             ///< complexMultiplier output buffer capacity (floats).

    cl_mem m_absBufferA{nullptr};              ///< Persistent absolute() input buffer.
    cl_mem m_absBufferC{nullptr};              ///< Persistent absolute() output buffer.
    size_t m_absBufferCapacityA{0};            ///< absolute() input buffer capacity (floats).
    size_t m_absBufferCapacityC{0};            ///< absolute() output buffer capacity (floats).

    cl_mem m_sumBufferInput{nullptr};          ///< Persistent sum() input buffer.
    cl_mem m_sumBufferOutput{nullptr};         ///< Persistent sum() partial-sum output buffer.
    size_t m_sumBufferInputCapacity{0};        ///< sum() input buffer capacity (floats).
    size_t m_sumBufferOutputCapacity{0};       ///< sum() output buffer capacity (floats, one per work-group).

    /** @brief Residency state of a numbered result slot. */
    using SlotStateType = enum SlotState : std::int8_t
    {
        SlotInvalid = 0,    ///< Slot holds no valid result.
        SlotDevice = 1,     ///< Slot result lives in the slot's device buffer.
        SlotHost = 2        ///< Slot result lives in the slot's host vector (CPU fallback).
    };

    cl_mem m_slotPoolBuffer{nullptr};              ///< Pooled slot device buffer, slots back to back.
    size_t m_slotPoolCapacity{0};                  ///< Slot pool capacity (floats).
    size_t m_slotPoolStrideFloats{0};              ///< Per-slot stride in the pool (floats).
    std::vector<ComplexFloatVector> m_slotHost;    ///< Per-slot host vectors for the CPU fallback.
    std::vector<SlotStateType> m_slotStates;       ///< Per-slot residency state.
    std::vector<unsigned int> m_slotLengths;       ///< Per-slot stored element counts.

    cl_mem m_batchWorkBuffer{nullptr};             ///< Batched multiply/FFT work buffer.
    size_t m_batchWorkCapacity{0};                 ///< Batch work buffer capacity (floats).
    cl_mem m_batchAbsBuffer{nullptr};              ///< Batched magnitude-squared output buffer.
    size_t m_batchAbsCapacity{0};                  ///< Batch abs buffer capacity (floats).
    cl_mem m_batchParamsBuffer{nullptr};           ///< Batched per-bin parameter buffer.
    size_t m_batchParamsCapacity{0};               ///< Batch params buffer capacity (floats).
    std::vector<cl_uint> m_batchParamsHost;        ///< Host staging for per-bin parameters.
    const void *m_cachedInput1Ptr{nullptr};        ///< Address of the device-cached resident-multiply input1.
    size_t m_cachedInput1Len{0};                   ///< Element count of the device-cached input1.
    ComplexFloatVector m_hostProductScratch;       ///< CPU-fallback scratch for the resident multiply product.
    ComplexFloatVector m_hostFftScratch;           ///< CPU-fallback scratch for the resident-multiply FFT result.
};
}

#endif
