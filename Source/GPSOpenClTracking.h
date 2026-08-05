#ifndef INCLUDED_GPSOPENCL_TRACKING_H
#define INCLUDED_GPSOPENCL_TRACKING_H

/** @file GPSOpenClTracking.h
 *  @brief PLL/DLL satellite tracking with Early/Prompt/Late correlators.
 */

#include <memory>
#include <utility>

#include "GPSOpenClCode.h"
#include "GPSOpenClSettings.h"
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
     *  @param channelState Owning channel state, for telemetry. */
    void doWork(const ComplexFloatVector &input, int prn, ComplexFloatVector *output, uint32_t channelState = 0);

    /** @brief Get current tracking results for a PRN.
     *  @param prn Satellite PRN.
     *  @return Tracking output struct. */
    TrackingOutput getTrackingOutput(int prn) const;

    /** @brief Get the latest doWork() sub-stage timings (ms). Values reflect only the last call.
     *   The correlator pass is fully fused. Its whole duration reports through accumulatorMs. The
     *   first two outputs are always zero.
     *  @param earlyLatePromptGenMs Output, always zero, stage is fused.
     *  @param numericOscillatorMs  Output, always zero, stage is fused.
     *  @param accumulatorMs        Output, fused correlator pass duration (ms). */
    void getSubStageTimings(float *earlyLatePromptGenMs, float *numericOscillatorMs, float *accumulatorMs) const;

    /** @brief Enable or disable timing samples in doWork(). A disabled profiler adds no clock
     *   overhead.
     *  @param enabled True to take timing samples. */
    void setTimingEnabled(bool enabled) { m_timingEnabled = enabled; }

    /** @brief Get smoothed carrier lock indicator.
     *  @return Value near +1 when phase-locked, near 0 when unlocked. */
    float getCarrierLockIndicator() const { return m_carrierLockEma; }

    /** @brief Get smoothed code lock ratio.
     *  @return Value near 1.0 when code phase is correctly aligned. */
    float getCodeLockRatio() const { return m_codeLockEma; }

    /** @brief Get the residual DLL code phase after this block. Difference consecutive readings mod
     *   1023 chips. That gives the accumulated code drift. It is the missing sub-millisecond
     *   transmit time correction.
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

    /** @brief Compute the cross and dot frequency discriminator. Uses consecutive prompt samples.
     *   Deliberately the four quadrant atan2(cross, dot) form. Its +-500 Hz pull-in range covers a
     *   full bin acquisition error. The two quadrant atan(cross/dot) form was tried and rejected.
     *   Its +-250 Hz range wraps a one bin error. The loop then converges 500 Hz away, measured.
     *   Nav bit flips inject occasional spikes. The FLL bandwidth averages them out.
     *  @param ipPrev Previous block in-phase prompt.
     *  @param qpPrev Previous block quadrature prompt.
     *  @param ip     Current block in-phase prompt.
     *  @param qp     Current block quadrature prompt.
     *  @return Frequency error estimate (Hz). */
    static float computeFllError(double ipPrev, double qpPrev, double ip, double qp);

  private:
    /** @brief Fused correlator pass in one loop. Generates replica indices and the NCO phasor
     *   incrementally. Wipes the carrier off each sample. Accumulates all six correlator sums. No
     *   intermediate buffers.
     *  @param input Raw IQ samples.
     *  @param prn   Satellite PRN. */
    void correlator(const ComplexFloatVector &input, int prn);

    /** @brief Compute the Costas phase discriminator from Prompt sums. It is insensitive to data
     *   bit sign. Uses the double angle atan2(2*Ip*Qp, Ip^2-Qp^2)/2 form. A nav bit flip changes
     *   only the Ip and Qp signs. A plain atan2(Qp,Ip) form would misread it.
     *  @return Phase error estimate (cycles, range (-0.25, 0.25]). */
    float computeCostasPhaseError() const;

    /** @brief Check if the prompt sum is trustworthy. Guards against nav bit transition blocks.
     *   Their correlation partially cancels. Reliable blocks update the running magnitude average.
     *  @return True if carrier discriminators should be applied. */
    bool isPromptSignalReliable();

    /** @brief Compute the FLL pull-in frequency discriminator. Runs only on reliable blocks.
     *   Unreliable blocks hold the carrier frequency steady. Corrupted sums never reach the FLL
     *   integrator. */
    void fllDiscriminator();

    /** @brief Bleed settled PLL NCO into the carrier basis. This re-centers the NCO. No second
     *   independent integrator is created. m_rateAidGain sets the bleed fraction per block.
     *   m_carrFreqBasisHz is clamped to +-15 kHz. */
    void rateAidDiscriminator();

    /** @brief Compute the PLL phase discriminator. Uses computeCostasPhaseError(). Advances the
     *   2nd order loop filter. */
    void freqDiscriminator();

    /** @brief Compute the DLL code discriminator. The DLL is carrier aided. Scaled carrier Doppler
     *   feeds the code frequency directly. The DLL corrects only the residual error. Normalized
     *   early minus late over sum has slope 2, not 1. So the result is scaled by 0.5. m_codeErrorChips
     *   then reports true chips of error. The loop filter formulas assume unity gain. Without the
     *   scale the noise bandwidth doubles. */
    void codeDiscriminator();

    /** @brief Reset correlator accumulators to zero. */
    void resetAccumulation();

    /** @brief Update smoothed carrier and code lock indicators. */
    void updateLockIndicators();

    Code m_code;                                ///< C/A code generator.
    Settings::Configuration m_configuration;    ///< Application configuration.
    TrackingInput m_inputConfig;                ///< Tracking parameters.

    int m_totalSamples;                         ///< Samples per code period.

    float m_pllTau1;                            ///< PLL loop filter tau1 (s).
    float m_pllTau2;                            ///< PLL loop filter tau2 (s).
    float m_carrFreqBasisHz;                    ///< Nominal carrier frequency (Hz).
    float m_carrFreqHz;                         ///< Current carrier frequency (Hz).
    float m_remCarrPhase;                       ///< Residual carrier phase (rad).
    float m_carrNco;                            ///< Carrier NCO output (Hz).
    float m_carrNcoPrev;                        ///< Previous carrier NCO output (Hz).
    float m_carrErrorCycles;                    ///< PLL phase error (rad).
    float m_carrErrorPrevCycles;                ///< Previous PLL phase error (rad).

    float m_fllGain;                            ///< FLL 1st-order loop filter gain (per block).
    float m_rateAidGain;                        ///< Continuous Doppler-rate-aiding gain (per block).
    float m_fllNco;                             ///< FLL NCO output (Hz).
    float m_ipPrev;                             ///< Previous block In-phase Prompt sum.
    float m_qpPrev;                             ///< Previous block Quadrature Prompt sum.
    float m_promptMagnitudeEma;     ///< Running average prompt correlator magnitude, for bit-transition gating.
    int m_blocksSinceInit;          ///< Blocks processed since last initTrackingState().
    int m_fllPullInBlocks;          ///< Blocks of FLL pull-in before PLL takes over.

    float m_dllTau1;                ///< DLL loop filter tau1 (s).
    float m_dllTau2;                ///< DLL loop filter tau2 (s).
    float m_codeFreqBasisHz;        ///< Nominal code frequency (Hz).
    float m_codeFreqHz;             ///< Current code frequency (Hz).
    float m_codePhaseStep;          ///< Code phase step per sample.
    float m_remCodePhase;           ///< Residual code phase (chips).
    float m_codeNco;                ///< Code NCO output (Hz).
    float m_codeNcoPrev;            ///< Previous code NCO output (Hz).
    float m_codeErrorChips;         ///< DLL code error (chips).
    float m_codeErrorPrevChips;     ///< Previous DLL code error (chips).

    float m_Ie;                     ///< In-phase Early accumulation.
    float m_Ip;                     ///< In-phase Prompt accumulation.
    float m_Il;                     ///< In-phase Late accumulation.
    float m_Qe;                     ///< Quadrature Early accumulation.
    float m_Qp;                     ///< Quadrature Prompt accumulation.
    float m_Ql;                     ///< Quadrature Late accumulation.

    float m_carrierLockEma;         ///< Smoothed carrier lock indicator.
    float m_codeLockEma;            ///< Smoothed code lock ratio.
    uint32_t m_lastChannelState;    ///< Owning channel state, for telemetry.

    bool m_timingEnabled{true};     ///< True to take correlator timing samples in doWork().
    float m_earlyLatePromptGenTimeMs{0.0f};    ///< Last doWork() earlyLatePromptGen duration (ms).
    float m_numericOscillatorTimeMs{0.0f};     ///< Last doWork() numericOscillator duration (ms).
    float m_accumulatorTimeMs{0.0f};           ///< Last doWork() accumulator duration (ms).
};
}
#endif
