#ifndef INCLUDED_GPSOPENCL_TRACKING_H
#define INCLUDED_GPSOPENCL_TRACKING_H

/** @file GPSOpenClTracking.h
 *  @brief PLL/DLL satellite tracking with Early/Prompt/Late correlators.
 */

#include <memory>
#include <utility>

#include "GPSOpenClCode.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClSink.h"
#include "GPSOpenClStructs.h"

namespace GPSOpenCl
{
/** @brief Code and carrier tracking engine (FLL-assisted 2nd-order Costas PLL + DLL). */
class Tracking
{
  public:
    /** @brief Construct from full configuration.
     *  @param conf Application configuration. */
    Tracking(const Settings::Configuration &conf);

    /** @brief Construct from tracking parameters.
     *  @param input Tracking settings. */
    Tracking(const TrackingInput &input);

    ~Tracking();
    Tracking(const Tracking &) = delete;
    Tracking &operator=(const Tracking &) = delete;
    Tracking(Tracking &&) = delete;
    Tracking &operator=(Tracking &&) = delete;

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

    /** @brief Get this instance's most recent doWork() sub-stage timings (ms), for aggregation into
     *   ProfilerOutput. Values reflect only the last call, not accumulated across blocks.
     *  @param earlyLatePromptGenMs Output: earlyLatePromptGen duration (ms).
     *  @param numericOscillatorMs  Output: numericOscillator duration (ms).
     *  @param accumulatorMs        Output: accumulator duration (ms). */
    void getSubStageTimings(float *earlyLatePromptGenMs, float *numericOscillatorMs, float *accumulatorMs) const;

    /** @brief Set telemetry sink.
     *  @param sink Sink implementation. */
    void setSink(std::shared_ptr<Sink> sink) { m_sink = std::move(sink); }

    /** @brief Get smoothed carrier lock indicator.
     *  @return Value near +1 when phase-locked, near 0 when unlocked. */
    float getCarrierLockIndicator() const { return m_carrierLockEma; }

    /** @brief Get smoothed code lock ratio.
     *  @return Value near 1.0 when code phase is correctly aligned. */
    float getCodeLockRatio() const { return m_codeLockEma; }

    /** @brief Get the residual DLL code phase left over after this block's whole-chip early/prompt/late
     *   generation. Consecutive readings, differenced and unwrapped mod 1023 chips, give the code-phase
     *   drift accumulated between them - the sub-millisecond correction the coarse block-count-based
     *   transmit-time estimate is missing.
     *  @return Code phase (chips, 0-1023). */
    float getCodePhaseChips() const { return m_remCodePhase; }

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

    /** @brief Compute the Costas (data-bit-sign-invariant) phase discriminator from the current
     *   Prompt correlator sum. Uses the double-angle atan2(2*Ip*Qp, Ip^2-Qp^2)/2 form so a genuine
     *   50 bps nav data-bit transition (which flips Ip/Qp's sign but not the true carrier phase)
     *   cannot be misread as a phase error, unlike a plain atan2(Qp,Ip) discriminator.
     *  @return Phase error estimate (cycles, range (-0.25, 0.25]). */
    float computeCostasPhaseError() const;

    /** @brief Check whether this block's prompt correlator sum is strong enough to trust for carrier
     *   discrimination, guarding against nav-bit-transition blocks whose correlation partially cancels.
     *   Updates the running magnitude average as a side effect when the block is judged reliable.
     *  @return True if this block's carrier discriminators should be applied. */
    bool isPromptSignalReliable();

    /** @brief Compute FLL-assisted pull-in frequency discriminator. Only invoked on blocks judged
     *   reliable by isPromptSignalReliable(); an unreliable block holds the carrier frequency steady
     *   instead of feeding a nav-bit-transition-corrupted correlator sum into the FLL integrator. */
    void fllDiscriminator();

    /** @brief Transfer a fraction of the settled PLL NCO into the carrier frequency basis,
     *   re-centering the NCO without creating a second independent integrator.
     *   m_rateAidGain controls the bleed fraction per block; m_carrFreqBasis is clamped
     *   to ±15 kHz to bound long-term drift. */
    void rateAidDiscriminator();

    /** @brief Compute PLL phase discriminator via computeCostasPhaseError() and advance the 2nd-order loop filter. */
    void freqDiscriminator();

    /** @brief Compute DLL code discriminator. Carrier-aided: the current carrier Doppler estimate
     *   (m_carrFreq) is scaled by GPS_L1_CARRIER_TO_CODE_RATIO and added directly to the code
     *   frequency, so the DLL's own loop filter only has to correct the residual code-phase error
     *   the carrier loop's Doppler estimate does not already account for.
     *   Normalized early-minus-late-over-sum, at +-0.5 chip spacing, has ACF slope 2 (not 1) at
     *   zero error: for a triangular ACF, E(e)-L(e) = -2e and E(e)+L(e) = 2-d with d = 1 chip
     *   separation, so raw (E-L)/(E+L) = -2e. The result is scaled by 0.5 so m_codeError reports
     *   the code-phase error itself (chips), matching the unity-gain assumption the loopFilterTau1/2
     *   noise-bandwidth formulas are derived under; without it dllBandwidthHz would configure roughly
     *   double the intended noise bandwidth. */
    void codeDiscriminator();

    /** @brief Reset correlator accumulators to zero. */
    void resetAccumulation();

    /** @brief Update smoothed carrier and code lock indicators. */
    void updateLockIndicators();

    Code m_code;                                ///< C/A code generator.
    Settings::Configuration m_configuration;    ///< Application configuration.
    TrackingInput m_inputConfig;                ///< Tracking parameters.
    std::shared_ptr<Sink> m_sink{nullptr};      ///< Telemetry sink.

    int m_totalSamples;                         ///< Samples per code period.

    ComplexFloatVector m_carrSig;               ///< Carrier-wiped signal buffer.
    float m_pllTau1;                            ///< PLL loop filter tau1 (s).
    float m_pllTau2;                            ///< PLL loop filter tau2 (s).
    float m_carrFreqBasis;                      ///< Nominal carrier frequency (Hz).
    float m_carrFreq;                           ///< Current carrier frequency (Hz).
    float m_remCarrPhase;                       ///< Residual carrier phase (rad).
    float m_carrNco;                            ///< Carrier NCO output (Hz).
    float m_carrNcoPrev;                        ///< Previous carrier NCO output (Hz).
    float m_carrError;                          ///< PLL phase error (rad).
    float m_carrErrorPrev;                      ///< Previous PLL phase error (rad).

    float m_fllGain;                            ///< FLL 1st-order loop filter gain (per block).
    float m_rateAidGain;                        ///< Continuous Doppler-rate-aiding gain (per block).
    float m_fllNco;                             ///< FLL NCO output (Hz).
    float m_ipPrev;                             ///< Previous block's In-phase Prompt sum.
    float m_qpPrev;                             ///< Previous block's Quadrature Prompt sum.
    float m_promptMagnitudeEma;     ///< Running average prompt correlator magnitude, for bit-transition gating.
    int m_blocksSinceInit;          ///< Blocks processed since last initTrackingState().
    int m_fllPullInBlocks;          ///< Blocks of FLL pull-in before PLL takes over.

    FloatVector m_earlyCode;        ///< Early code replica.
    FloatVector m_promptCode;       ///< Prompt code replica.
    FloatVector m_lateCode;         ///< Late code replica.
    float m_dllTau1;                ///< DLL loop filter tau1 (s).
    float m_dllTau2;                ///< DLL loop filter tau2 (s).
    float m_codeFreqBasis;          ///< Nominal code frequency (Hz).
    float m_codeFreq;               ///< Current code frequency (Hz).
    float m_codePhaseStep;          ///< Code phase step per sample.
    float m_remCodePhase;           ///< Residual code phase (chips).
    float m_codeNco;                ///< Code NCO output (Hz).
    float m_codeNcoPrev;            ///< Previous code NCO output (Hz).
    float m_codeError;              ///< DLL code error (chips).
    float m_codeErrorPrev;          ///< Previous DLL code error (chips).

    float m_Ie;                     ///< In-phase Early accumulation.
    float m_Ip;                     ///< In-phase Prompt accumulation.
    float m_Il;                     ///< In-phase Late accumulation.
    float m_Qe;                     ///< Quadrature Early accumulation.
    float m_Qp;                     ///< Quadrature Prompt accumulation.
    float m_Ql;                     ///< Quadrature Late accumulation.

    float m_carrierLockEma;         ///< Smoothed carrier lock indicator.
    float m_codeLockEma;            ///< Smoothed code lock ratio.
    uint32_t m_lastChannelState;    ///< Owning channel's state, for telemetry.

    float m_earlyLatePromptGenTimeMs{0.0f};    ///< Last doWork() call's earlyLatePromptGen duration (ms).
    float m_numericOscillatorTimeMs{0.0f};     ///< Last doWork() call's numericOscillator duration (ms).
    float m_accumulatorTimeMs{0.0f};           ///< Last doWork() call's accumulator duration (ms).
};
}
#endif
