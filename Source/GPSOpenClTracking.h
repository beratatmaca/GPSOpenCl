#ifndef INCLUDED_GPSOPENCL_TRACKING_H
#define INCLUDED_GPSOPENCL_TRACKING_H

/** @file GPSOpenClTracking.h
 *  @brief PLL/DLL satellite tracking with Early/Prompt/Late correlators.
 */

#include <memory>

#include "GPSOpenClCode.h"
#include "GPSOpenClGPUCompute.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClSink.h"
#include "GPSOpenClStructs.h"

namespace GPSOpenCl
{
/** @brief Code and carrier tracking engine (FLL-assisted 3rd-order PLL + DLL). */
class Tracking
{
  public:
    /** @brief Construct from full configuration.
     *  @param conf Application configuration. */
    Tracking(Settings::Configuration conf);

    /** @brief Construct from tracking parameters.
     *  @param input Tracking settings. */
    Tracking(const TrackingInput &input);

    ~Tracking();

    /** @brief Set initial Doppler and code phase from acquisition.
     *  @param initDopplerHz      Acquired Doppler shift (Hz).
     *  @param initCodePhaseChips Acquired code phase (chips). */
    void initTrackingState(float initDopplerHz, float initCodePhaseChips);

    /** @brief Process one code period of samples.
     *  @param input        IQ samples.
     *  @param prn          Satellite PRN.
     *  @param output       Carrier-wiped output.
     *  @param channelState Owning channel's state, for telemetry. */
    void doWork(const ComplexFloatVector &input, int prn, ComplexFloatVector *output, uint32_t channelState = 0);

    /** @brief Get current tracking results for a PRN.
     *  @param prn Satellite PRN.
     *  @return Tracking output struct. */
    TrackingOutput getTrackingOutput(int prn) const;

    /** @brief Multiply input samples by NCO carrier replica.
     *  @param input     IQ samples.
     *  @param frequency NCO frequency (Hz).
     *  @param output    Carrier-wiped output. */
    void ncoMultiplicate(const ComplexFloatVector &input, float frequency, ComplexFloatVector *output);

    /** @brief Set telemetry sink.
     *  @param sink Sink implementation. */
    void setSink(std::shared_ptr<Sink> sink) { m_sink = sink; }

    /** @brief Get smoothed carrier lock indicator.
     *  @return Value near +1 when phase-locked, near 0 when unlocked. */
    float getCarrierLockIndicator() const { return m_carrierLockEma; }

    /** @brief Get smoothed code lock ratio.
     *  @return Value near 1.0 when code phase is correctly aligned. */
    float getCodeLockRatio() const { return m_codeLockEma; }

    /** @brief Compute PLL loop filter tau1 time constant.
     *  @param noiseBandwidthHz Loop noise bandwidth (Hz).
     *  @return Tau1 value (s). */
    static float loopFilterTau1(double noiseBandwidthHz);

    /** @brief Compute PLL loop filter tau2 time constant.
     *  @param noiseBandwidthHz Loop noise bandwidth (Hz).
     *  @return Tau2 value (s). */
    static float loopFilterTau2(double noiseBandwidthHz);

  private:
    /** @brief Generate Early/Prompt/Late code replicas.
     *  @param prn Satellite PRN. */
    void earlyLatePromptGen(int prn);

    /** @brief Generate NCO phase ramp. */
    void numericOscillator();

    /** @brief Accumulate correlator outputs.
     *  @param input Carrier-wiped IQ samples. */
    void accumulator(const ComplexFloatVector &input);

    /** @brief Compute the cross/dot frequency discriminator from consecutive prompt correlator samples.
     *  @return Frequency error estimate (Hz). */
    float computeFllError() const;

    /** @brief Check whether this block's prompt correlator sum is strong enough to trust for carrier
     *   discrimination, guarding against nav-bit-transition blocks whose correlation partially cancels.
     *   Updates the running magnitude average as a side effect when the block is judged reliable.
     *  @return True if this block's carrier discriminators should be applied. */
    bool isPromptSignalReliable();

    /** @brief Compute FLL-assisted pull-in frequency discriminator. */
    void fllDiscriminator();

    /** @brief Transfer a fraction of the settled PLL NCO into the carrier frequency basis,
     *   re-centering the NCO without creating a second independent integrator.
     *   m_rateAidGain controls the bleed fraction per block; m_carrFreqBasis is clamped
     *   to ±15 kHz to bound long-term drift. */
    void rateAidDiscriminator();

    /** @brief Compute PLL frequency discriminator. */
    void freqDiscriminator();

    /** @brief Compute DLL code discriminator. */
    void codeDiscriminator();

    /** @brief Reset correlator accumulators to zero. */
    void resetAccumulation();

    /** @brief Update smoothed carrier and code lock indicators. */
    void updateLockIndicators();

    Code m_code;                              ///< C/A code generator.
    Compute *m_gpu;                           ///< GPU/CPU compute back-end.
    Settings::Configuration m_configuration;  ///< Application configuration.
    TrackingInput m_inputConfig;              ///< Tracking parameters.
    std::shared_ptr<Sink> m_sink{nullptr};    ///< Telemetry sink.

    int m_totalSamples;                       ///< Samples per code period.

    ComplexFloatVector m_carrSig;             ///< Carrier-wiped signal buffer.
    float m_pllTau1;                          ///< PLL loop filter tau1 (s).
    float m_pllTau2;                          ///< PLL loop filter tau2 (s).
    float m_carrFreqBasis;                    ///< Nominal carrier frequency (Hz).
    float m_carrFreq;                         ///< Current carrier frequency (Hz).
    float m_remCarrPhase;                     ///< Residual carrier phase (rad).
    float m_carrNco;                          ///< Carrier NCO output (Hz).
    float m_carrNcoPrev;                      ///< Previous carrier NCO output (Hz).
    float m_carrError;                        ///< PLL phase error (rad).
    float m_carrErrorPrev;                    ///< Previous PLL phase error (rad).

    float m_fllGain;                          ///< FLL 1st-order loop filter gain (per block).
    float m_rateAidGain;                      ///< Continuous Doppler-rate-aiding gain (per block).
    float m_fllNco;                           ///< FLL NCO output (Hz).
    float m_ipPrev;                           ///< Previous block's In-phase Prompt sum.
    float m_qpPrev;                           ///< Previous block's Quadrature Prompt sum.
    float m_promptMagnitudeEma;                ///< Running average prompt correlator magnitude, for bit-transition gating.
    int m_blocksSinceInit;                    ///< Blocks processed since last initTrackingState().
    int m_fllPullInBlocks;                    ///< Blocks of FLL pull-in before PLL takes over.

    FloatVector m_earlyCode;                  ///< Early code replica.
    FloatVector m_promptCode;                 ///< Prompt code replica.
    FloatVector m_lateCode;                   ///< Late code replica.
    float m_dllTau1;                          ///< DLL loop filter tau1 (s).
    float m_dllTau2;                          ///< DLL loop filter tau2 (s).
    float m_codeFreqBasis;                    ///< Nominal code frequency (Hz).
    float m_codeFreq;                         ///< Current code frequency (Hz).
    float m_codePhaseStep;                    ///< Code phase step per sample.
    float m_remCodePhase;                     ///< Residual code phase (chips).
    float m_codeNco;                          ///< Code NCO output (Hz).
    float m_codeNcoPrev;                      ///< Previous code NCO output (Hz).
    float m_codeError;                        ///< DLL code error (chips).
    float m_codeErrorPrev;                    ///< Previous DLL code error (chips).

    float m_Ie;                               ///< In-phase Early accumulation.
    float m_Ip;                               ///< In-phase Prompt accumulation.
    float m_Il;                               ///< In-phase Late accumulation.
    float m_Qe;                               ///< Quadrature Early accumulation.
    float m_Qp;                               ///< Quadrature Prompt accumulation.
    float m_Ql;                               ///< Quadrature Late accumulation.

    float m_carrierLockEma;                   ///< Smoothed carrier lock indicator.
    float m_codeLockEma;                      ///< Smoothed code lock ratio.
    uint32_t m_lastChannelState;              ///< Owning channel's state, for telemetry.
};
}
#endif
