#ifndef INCLUDED_GPSOPENCL_TRACKING_H
#define INCLUDED_GPSOPENCL_TRACKING_H

#include <memory>

#include "GPSOpenClCode.h"
#include "GPSOpenClGPUCompute.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClSink.h"
#include "GPSOpenClStructs.h"

namespace GPSOpenCl
{
class Tracking
{
  public:
    Tracking(Settings::Configuration conf);
    Tracking(const TrackingInput &input);
    ~Tracking();

    void initTrackingState(float initDopplerHz, float initCodePhaseChips);
    void doWork(const ComplexFloatVector &input, int prn, ComplexFloatVector *output);
    TrackingOutput getTrackingOutput(int prn) const;
    void ncoMultiplicate(const ComplexFloatVector &input, float frequency, ComplexFloatVector *output);

    void setSink(std::shared_ptr<Sink> sink) { m_sink = sink; }

    // 2nd-order loop filter time constants (tau1, tau2) for a given noise bandwidth, damping
    // ratio 1/sqrt(2) and unity loop gain (Kaplan/Borre standard PLL/DLL loop filter design).
    static float loopFilterTau1(double noiseBandwidthHz);
    static float loopFilterTau2(double noiseBandwidthHz);

  private:
    void earlyLatePromptGen(int prn);
    void numericOscillator();
    void accumulator(const ComplexFloatVector &input);
    void freqDiscriminator();
    void codeDiscriminator();
    void resetAccumulation();

    Code m_code;
    Compute *m_gpu;
    Settings::Configuration m_configuration;
    TrackingInput m_inputConfig;
    std::shared_ptr<Sink> m_sink{nullptr};

    int m_totalSamples;

    ComplexFloatVector m_carrSig;
    float m_pllTau1;
    float m_pllTau2;
    float m_carrFreqBasis;
    float m_carrFreq;
    float m_remCarrPhase;
    float m_carrNco;
    float m_carrNcoPrev;
    float m_carrError;
    float m_carrErrorPrev;

    FloatVector m_earlyCode;
    FloatVector m_promptCode;
    FloatVector m_lateCode;
    float m_dllTau1;
    float m_dllTau2;
    float m_codeFreqBasis;
    float m_codeFreq;
    float m_codePhaseStep;
    float m_remCodePhase;
    float m_codeNco;
    float m_codeNcoPrev;
    float m_codeError;
    float m_codeErrorPrev;

    float m_Ie;
    float m_Ip;
    float m_Il;
    float m_Qe;
    float m_Qp;
    float m_Ql;
};
} // namespace GPSOpenCl
#endif //! INCLUDED_GPSOPENCL_TRACKING_H
