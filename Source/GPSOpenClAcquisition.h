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
    Acquisition(Settings::Configuration conf);

    /** @brief Construct from acquisition parameters.
     *  @param input Acquisition settings. */
    Acquisition(const AcquisitionInput &input);

    ~Acquisition();

    /** @brief Run acquisition for one satellite channel.
     *  @param input      IQ samples for one code period.
     *  @param gpu        Compute back-end (GPU or CPU).
     *  @param code       C/A code lookup table.
     *  @param acqChannel Target channel for results. */
    void correlate(const ComplexFloatVector &input, Compute *gpu, Code *code, Channel *acqChannel);

  private:
    /** @brief Build the Doppler frequency search table. */
    void createDopplerSearchTable();

    /** @brief Generate complex exponential (carrier replica).
     *  @param length       Number of samples.
     *  @param frequency    Frequency (Hz).
     *  @param samplingRate Sampling rate (Hz).
     *  @param phaseOffset  Initial phase (rad).
     *  @param output       Output complex vector. */
    void exp(int length, float frequency, float samplingRate, float phaseOffset, ComplexFloatVector *output);

    AcquisitionInput m_inputConfig;                 ///< Input parameters.
    std::vector<ComplexFloatVector> m_dopplerSearch; ///< Pre-computed Doppler replicas.
    int m_numberOfFreqencyBins;                     ///< Number of Doppler bins.
    float m_initialFrequency;                       ///< First Doppler bin frequency (Hz).
    float m_freqSpacing;                            ///< Doppler bin spacing (Hz).
    int m_length;                                   ///< Samples per code period.
    float m_samplingFrequency;                      ///< Sampling rate (Hz).
};
}

#endif
