#ifndef INCLUDED_GPSOPENCL_ACQUISITION_H
#define INCLUDED_GPSOPENCL_ACQUISITION_H

/** @file GPSOpenClAcquisition.h
 *  @brief FFT-based GPS satellite acquisition engine.
 */

#include <algorithm>
#include <vector>

#include "GPSOpenClChannel.h"
#include "GPSOpenClCode.h"
#include "GPSOpenClCommon.h"
#include "GPSOpenClGPUCompute.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClStructs.h"

namespace GPSOpenCl
{
/** @brief Satellite acquisition via Doppler search and FFT cross-correlation. */
class Acquisition
{
  public:
    /** @brief Construct from full configuration.
     *  @param conf Application configuration. */
    Acquisition(const Settings::Configuration &conf);

    /** @brief Construct from acquisition parameters.
     *  @param input Acquisition settings. */
    Acquisition(const AcquisitionInput &input);

    ~Acquisition();
    Acquisition(const Acquisition &) = delete;
    Acquisition &operator=(const Acquisition &) = delete;
    Acquisition(Acquisition &&) = delete;
    Acquisition &operator=(Acquisition &&) = delete;

    /** @brief Run acquisition for one satellite channel.
     *  @param input      IQ samples for one code period.
     *  @param gpu        Compute back-end (GPU or CPU).
     *  @param code       C/A code lookup table.
     *  @param acqChannel Target channel for results. */
    void correlate(const ComplexFloatVector &input, Compute *gpu, Code *code, Channel *acqChannel);

    /** @brief Get the number of forward-FFT reference spectra computed per correlate() call: the
     *   bin-resolution to search-spacing ratio when the spacing divides the FFT bin resolution
     *   exactly, or one spectrum per Doppler bin (no shift reuse) otherwise.
     *  @return Reference spectrum count, >= 1. */
    int getReuseFactor() const { return m_reuseFactor; }

  private:
    /** @brief Build the Doppler frequency search table. */
    void createDopplerSearchTable();

    /** @brief Generate complex exponential (carrier replica).
     *  @param length       Number of samples.
     *  @param frequency    Frequency (Hz).
     *  @param samplingRate Sampling rate (Hz).
     *  @param phaseOffset  Initial phase (rad).
     *  @param output       Output complex vector. */
    static void exp(int length, float frequency, float samplingRate, float phaseOffset, ComplexFloatVector *output);

    /** @brief Determine how many distinct forward-FFT "reference" spectra are needed to reach every
     *   searched Doppler bin via an exact circular frequency-domain shift, instead of one forward FFT
     *   per bin. Exact (not an approximation) whenever the Doppler bin spacing evenly divides the FFT's
     *   own bin resolution (samplingFrequency / numberOfSamplesPerCode): shifting a time-domain signal by
     *   exp(j*2*pi*m*n/N) for integer m is identical, in exact arithmetic, to circularly rotating its
     *   N-point DFT by m bins. Falls back to one reference per Doppler bin (shift always zero, one
     *   forward FFT per bin, matching the original algorithm exactly) whenever that divisibility
     *   doesn't hold, so correctness never depends on the configured search spacing. A fallback of 1
     *   would be wrong: in correlate() a reuse factor of 1 means every bin is derived by shifting a
     *   single reference, which is only exact when the spacing equals the bin resolution.
     *  @return Number of reference spectra, >= 1. */
    int computeReuseFactor() const;

    AcquisitionInput m_inputConfig;                     ///< Input parameters.
    std::vector<ComplexFloatVector> m_dopplerSearch;    ///< Pre-computed Doppler replicas.
    int m_numberOfFreqencyBins;                         ///< Number of Doppler bins.
    float m_initialFrequency;                           ///< First Doppler bin frequency (Hz).
    float m_freqSpacing;                                ///< Doppler bin spacing (Hz).
    int m_length;                                       ///< Samples per code period.
    float m_samplingFrequency;                          ///< Sampling rate (Hz).
    int m_reuseFactor;    ///< Number of forward-FFT reference spectra (see computeReuseFactor).
};
}

#endif
