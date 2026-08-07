#ifndef INCLUDED_GPSOPENCL_COMPUTE_HPP
#define INCLUDED_GPSOPENCL_COMPUTE_HPP

/** @file GPSOpenClSpectrumEngine.hpp
 *  @brief FFT, complex math, and NCO compute back-end.
 */

#include "Common/GPSOpenClCommon.hpp"
#include "Gpu/GPSOpenClGPUHandler.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace GPSOpenCl
{
/** @brief GPU/CPU compute engine for FFT and signal math.
 *
 *   Slot pool lifecycle. Acquisition keeps one reference spectrum per
 *   Doppler residue in a numbered slot. complexMultiplyThenFftToSlot
 *   fills a slot. On GPU the result stays device resident, state
 *   SlotDevice. On CPU fallback it lands in a host vector, state
 *   SlotHost. complexMultiplyResidentThenFftThenAbsolute then reuses a
 *   filled slot once per Doppler bin. A circular index shift replaces
 *   any host side copy or re upload. The batch variant submits all
 *   bins in one kernel launch with a single readback. Any error marks
 *   the slot SlotInvalid and the caller retries or falls back.
 *
 *   Every GPU entry point has a CPU path with identical math. A
 *   machine without OpenCL produces the same results. */
class SpectrumEngine
{
  public:
    SpectrumEngine();
    ~SpectrumEngine();
    SpectrumEngine(const SpectrumEngine &) = delete;
    SpectrumEngine &operator=(const SpectrumEngine &) = delete;
    SpectrumEngine(SpectrumEngine &&) = delete;
    SpectrumEngine &operator=(SpectrumEngine &&) = delete;

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

    /** @brief Radix 2 FFT on the CPU. Fallback path when no device exists.
     *  @param input     Input samples. Length must be a power of two.
     *  @param output    Transformed samples.
     *  @param direction Forward or inverse.
     *  @return 0 on success, -1 on invalid length. */
    static int fftCpu(const ComplexFloatVector &input, ComplexFloatVector *output, FFTDirectionType direction);

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

    /** @brief Complex multiply then FFT into a numbered slot. The result stays resident. GPU path
     *   uses a device buffer. CPU fallback uses a host vector. Pair with
     *   complexMultiplyResidentThenFftThenAbsolute() for reuse without host round trips.
     *  @param input1    First complex vector.
     *  @param input2    Second complex vector.
     *  @param direction Forward or inverse FFT.
     *  @param slot      Slot index to store the result in (grown on demand).
     *  @return 0 on success. */
    int complexMultiplyThenFftToSlot(const ComplexFloatVector &input1, const ComplexFloatVector &input2, FFTDirectionType direction, int slot);

    /** @brief Forget the device-resident input1 cache. The cache keys on the host pointer, so a
     *   reused buffer with new contents at the same address would otherwise be mistaken for the
     *   cached upload. Call before a batch whose input1 storage may have been recycled. */
    void invalidateResidentInput()
    {
        m_cachedInput1Ptr = nullptr;
        m_cachedInput1Len = 0;
    }

    /** @brief Multiply input1 with a shifted slot vector. Then FFT, then magnitude squared.
     *   Computes output = |FFT(input1[i] * slot[(i + length - shiftBins) mod length])|^2. The shift
     *   is applied through indexing. No shifted copy is created. input1 stays cached on the device.
     *   A different input1 triggers a re-upload. Callers must not change input1 storage between
     *   calls.
     *  @param input1    First complex vector (e.g. a precomputed constant code spectrum).
     *  @param slot      Slot index previously filled by complexMultiplyThenFftToSlot().
     *  @param shiftBins Circular shift applied to the slot vector, in [0, length).
     *  @param direction Forward or inverse FFT.
     *  @param output    Magnitude squared of the FFT of the product.
     *  @return 0 on success. */
    int complexMultiplyResidentThenFftThenAbsolute(const ComplexFloatVector &input1, int slot, int shiftBins, FFTDirectionType direction, FloatVector *output);

    /** @brief Batched form of complexMultiplyResidentThenFftThenAbsolute(). All bins run in one
     *   GPU submission. One readback at the end. Bin k occupies output[k * length, (k + 1) *
     *   length). Every slot must be resident with input1 length. Returns nonzero when the batch
     *   cannot run. Output is untouched then. Callers fall back to per-bin calls.
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

    /** @brief Complex points that fit the device local memory, minus a safety reserve. Some
     *   devices keep part of the reported local memory for themselves, and requesting all of it
     *   fails at enqueue.
     *  @param localMemoryBytes Reported device local memory (bytes).
     *  @return Complex points usable per work-group. */
    static unsigned int usableLocalMemoryPoints(unsigned long long localMemoryBytes);

    /** @brief Shrink a work-group size until pointsPerGroup/localSize meets a minimum.
     *  @param localSize        Work-group size, assumed already a power of two.
     *  @param pointsPerGroup   Points processed per work-group.
     *  @param minPointsPerItem Minimum points each work-item must process.
     *  @return Clamped work-group size. */
    static unsigned int clampLocalSizeForMinPointsPerItem(unsigned int localSize, unsigned int pointsPerGroup, unsigned int minPointsPerItem);

  private:
    /** @brief Query and cache per-kernel work-group size and device local memory size, once. */
    void cacheDeviceInfo();

    /** @brief Return a persistent device buffer. Holds at least neededFloats floats. Grows only
     *   when capacity is insufficient.
     *  @param buffer         Persistent buffer handle (in/out).
     *  @param capacityFloats Current buffer capacity in floats (in/out).
     *  @param neededFloats   Required capacity in floats.
     *  @return Buffer handle, or nullptr on allocation failure. */
    cl_mem ensureBuffer(cl_mem &buffer, size_t &capacityFloats, size_t neededFloats);

    /** @brief GPU complexMultiplier stage. The product stays on the device. Used by
     *   complexMultiplier() and chained calls.
     *  @param input1 First complex vector.
     *  @param input2 Second complex vector.
     *  @param length Element count (equal for both inputs).
     *  @return The device output buffer (m_cmBufferC), or nullptr on failure. */
    cl_mem complexMultiplierDevice(const ComplexFloatVector &input1, const ComplexFloatVector &input2, unsigned int length);

    /** @brief GPU complexMultiplier stage with a resident second operand. Reads it with a circular
     *   offset. input1 uploads only when it changed.
     *  @param input1 First complex vector (cached on-device between calls).
     *  @param input2 Device buffer holding length interleaved complex floats.
     *  @param length Element count (equal for both inputs).
     *  @param offset Circular read offset into input2, in [0, length).
     *  @return The device output buffer (m_cmBufferC), or nullptr on failure. */
    cl_mem complexMultiplierResidentDevice(const ComplexFloatVector &input1, cl_mem input2, unsigned int length, unsigned int offset);

    /** @brief Enqueue the complexMultiplier kernel on already-populated device buffers.
     *  @param inputA     First operand device buffer.
     *  @param inputB     Second operand device buffer, read with the circular offset.
     *  @param length     Element count.
     *  @param offset     Circular read offset into inputB, in [0, length).
     *  @param inputBBase Element offset of the operand vector inside inputB.
     *  @return The device output buffer (m_cmBufferC), or nullptr on failure. */
    cl_mem enqueueComplexMultiplier(cl_mem inputA, cl_mem inputB, unsigned int length, unsigned int offset, unsigned int inputBBase);

    /** @brief Upload input1 into the persistent input buffer. Skips the upload when input1 is
     *   already resident.
     *  @param input1 Complex vector to make device-resident.
     *  @param length Element count.
     *  @return The input device buffer (m_cmBufferA), or nullptr on failure. */
    cl_mem ensureResidentInput1(const ComplexFloatVector &input1, unsigned int length);

    /** @brief GPU FFT stage over pooled equal-length FFTs. FFTs sit back to back in one buffer.
     *   One work group per FFT, one launch. A whole FFT must fit in local memory.
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

    /** @brief Ensure the slot pool fits the given slot. Slots sit back to back in one buffer.
     *   Batched kernels address them by element offset. Growing the pool discards previous
     *   contents. Every slot state is invalidated then.
     *  @param slot          Slot index.
     *  @param floatsPerSlot Per-slot capacity in floats.
     *  @return Pool device buffer, or nullptr on allocation failure. */
    cl_mem ensureSlotPool(int slot, size_t floatsPerSlot);

    /** @brief GPU FFT stage in place on a device buffer. No host upload or readback. Used by fft()
     *   and chained calls.
     *  @param buffer    Device buffer holding length interleaved complex floats (in/out).
     *  @param length    Element count.
     *  @param direction Forward or inverse FFT.
     *  @return 0 on success. */
    int fftDeviceInPlace(cl_mem buffer, unsigned int length, FFTDirectionType direction);

    /** @brief GPU absolute stage reading a device buffer. No host upload. Used by absolute() and
     *   chained calls.
     *  @param inputBuffer Device buffer holding length interleaved complex floats.
     *  @param length      Element count.
     *  @param output      Magnitude squared output (host).
     *  @return 0 on success. */
    int absoluteDeviceToHost(cl_mem inputBuffer, unsigned int length, FloatVector *output);

    GpuHandler m_gpu;                          ///< OpenCL handler.
    cl_command_queue m_queue{nullptr};         ///< OpenCL command queue.
    cl_int m_error{-1};                        ///< Last OpenCL error.
    std::vector<float> m_allocatedMemory;      ///< Scratch memory buffer.

    bool m_deviceInfoCached{false};            ///< True once per-kernel/device info below is cached.
    bool m_batchPathUnavailable{false};        ///< Latched when the batch path structurally cannot run.
    cl_ulong m_localMemorySize{0};             ///< Cached device local memory size (bytes).
    size_t m_fftLocalSize{0};                  ///< Cached fft_init kernel work-group size.
    size_t m_complexMultiplierLocalSize{0};    ///< Cached complexMultiplier kernel work-group size.
    size_t m_absoluteLocalSize{0};             ///< Cached absolute kernel work-group size.

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

    /** @brief Residency state of a numbered result slot. */
    using SlotStateType = enum SlotState : std::int8_t
    {
        SlotInvalid = 0,    ///< Slot holds no valid result.
        SlotDevice = 1,     ///< Slot result lives in the slot device buffer.
        SlotHost = 2        ///< Slot result lives in the slot host vector.
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
