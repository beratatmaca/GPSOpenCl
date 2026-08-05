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

    /** @brief Run acquisition for one satellite channel. One reference spectrum per residue class
     *   stays slot resident. Other bins derive from it by circular shift. The shift is an index
     *   offset in the multiply. No host side shift, copy, or re-upload. All bins run as one GPU
     *   submission with one readback. Per bin round trips are the fallback.
     *  @param input      IQ samples for one code period.
     *  @param gpu        Compute back-end (GPU or CPU).
     *  @param code       C/A code lookup table.
     *  @param acqChannel Target channel for results. */
    void correlate(const ComplexFloatVector &input, Compute *gpu, Code *code, Channel *acqChannel);

    /** @brief Get the reference spectrum count per correlate() call. Equals the bin resolution over
     *   search spacing ratio. Requires exact divisibility. Otherwise one spectrum per Doppler bin.
     *  @return Reference spectrum count, >= 1. */
    int getReuseFactor() const { return m_reuseFactor; }

  private:
    /** @brief Build the Doppler frequency search table. */
    void createDopplerSearchTable();

    /** @brief Generate complex exponential (carrier replica).
     *  @param length       Number of samples.
     *  @param frequency    Frequency (Hz).
     *  @param samplingRateHz Sampling rate (Hz).
     *  @param phaseOffset  Initial phase (rad).
     *  @param output       Output complex vector. */
    static void exp(int length, float frequency, float samplingRateHz, float phaseOffset, ComplexFloatVector *output);

    /** @brief Count reference spectra needed for shift reuse. Each searched bin derives by exact
     *   circular shift. Exact when spacing divides the FFT bin resolution. A time shift by
     *   exp(j*2*pi*m*n/N) rotates the DFT m bins. Otherwise fall back to one reference per bin. The
     *   fallback shift is always zero. A fallback of 1 would be wrong. Factor 1 makes correlate()
     *   shift a single reference. That is exact only at bin resolution spacing.
     *  @return Number of reference spectra, >= 1. */
    int computeReuseFactor() const;

    AcquisitionInput m_inputConfig;                     ///< Input parameters.
    std::vector<ComplexFloatVector> m_dopplerSearch;    ///< Pre-computed Doppler replicas.
    int m_numberOfFreqencyBins;                         ///< Number of Doppler bins.
    float m_initialFrequencyHz;                         ///< First Doppler bin frequency (Hz).
    float m_freqSpacingHz;                              ///< Doppler bin spacing (Hz).
    int m_length;                                       ///< Samples per code period.
    float m_samplingFrequencyHz;                        ///< Sampling rate (Hz).
    int m_reuseFactor;    ///< Number of forward-FFT reference spectra (see computeReuseFactor).
};
}

#endif
