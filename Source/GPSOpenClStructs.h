#ifndef INCLUDED_GPSOPENCL_STRUCTS_H
#define INCLUDED_GPSOPENCL_STRUCTS_H

/** @file GPSOpenClStructs.h
 *  @brief Packed binary structs for inter-module data exchange.
 */

#include <cstdint>

/*
 * Wire protocol. Every output struct below is one telemetry message.
 * ZMQ transport sends two frames per message. Frame one is the struct
 * name as ASCII text. Frame two is the raw struct bytes. FileSink logs
 * the same messages as length-prefixed records. Record layout is
 * nameLen (u32), name bytes, dataLen (u32), struct bytes. All fields
 * are little-endian and packed with no padding. structVersion is always
 * the first field. New fields append at the end and bump the version,
 * so old readers can still parse the leading fields. The golden sizeof
 * assertions in Tests/StructsSourceSinkTest.cpp and the parser table in
 * Tools/dashboard.py must both match any struct change here.
 */

namespace GPSOpenCl
{

#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "Wire structs are written in host byte order and the protocol is little-endian"
#endif

#pragma pack(push, 1)

/** @brief Struct wire-format version tag. */
constexpr uint32_t STRUCT_VERSION_1 = 1;

/** @brief Wire format version 2. Adds timing and PVT quality fields. Applies to AcquisitionOutput,
 *   TrackingOutput, PvtSolverOutput. New fields are appended only. Version 1 consumers still parse
 *   the leading fields. */
constexpr uint32_t STRUCT_VERSION_2 = 2;

/** @brief Source module input parameters. */
struct SourceInput
{
    uint32_t structVersion{STRUCT_VERSION_1};    ///< Struct version tag.
    char fifoPath[256]{};                        ///< FIFO or file path.
    double samplingRateHz{4096000.0};            ///< Sampling rate (Hz).
};

/** @brief Source module output telemetry. */
struct SourceOutput
{
    uint32_t structVersion{STRUCT_VERSION_1};    ///< Struct version tag.
    uint32_t blockIndex{0};                      ///< Current block index.
    double timestampSec{0.0};                    ///< Block timestamp in seconds.
    uint32_t fifoUnderrunCount{0};               ///< FIFO underrun count.
    uint32_t fifoOverrunCount{0};                ///< FIFO overrun count.
};

/** @brief Acquisition module input parameters. */
struct AcquisitionInput
{
    uint32_t structVersion{STRUCT_VERSION_1};      ///< Struct version tag.
    int32_t acquisitionDopplerMinimum{-4000};      ///< Min Doppler search (Hz).
    int32_t acquisitionDopplerMaximum{4000};       ///< Max Doppler search (Hz).
    int32_t acquisitionDopplerSearchRange{500};    ///< Doppler bin step (Hz).
    double samplingFrequencyHz{4096000.0};         ///< Sampling rate (Hz).
    int32_t numberOfSamplesPerCode{4096};          ///< Samples per code period.
    int32_t reacquisitionIntervalBlocks{1000};     ///< Blocks between re-acquisition attempts.
    double acquisitionCn0ThresholdDbHz{43.0};      ///< Min C/N0 (dB-Hz) to declare a satellite acquired.
};

/** @brief Acquisition module output results. */
struct AcquisitionOutput
{
    uint32_t structVersion{STRUCT_VERSION_2};    ///< Struct version tag.
    int32_t prn{0};                              ///< Satellite PRN number.
    int32_t peakIndex{0};                        ///< Code phase of peak (samples).
    double peakValue{0.0};                       ///< Correlation peak magnitude.
    double peakFrequencyHz{0.0};                 ///< Doppler at peak (Hz).
    double meanValue{0.0};                       ///< Mean correlation level.
    double cnoDbHz{0.0};                         ///< Carrier-to-noise ratio (dB-Hz).
    double peakRatio{0.0};                       ///< Peak-to-mean ratio.
    uint32_t isAcquired{0};                      ///< 1 if satellite acquired.
    double correlateMs{0.0};                     ///< Wall-clock duration of the correlate() call (ms).
};

/** @brief Tracking module input parameters. */
struct TrackingInput
{
    uint32_t structVersion{STRUCT_VERSION_1};    ///< Struct version tag.
    double pllBandwidthHz{25.0};                 ///< PLL noise bandwidth (Hz).
    double dllBandwidthHz{2.0};                  ///< DLL noise bandwidth (Hz).
    double fllBandwidthHz{10.0};                 ///< FLL pull-in noise bandwidth (Hz).
    int32_t fllPullInBlocks{75};                 ///< Blocks of FLL-assisted pull-in before PLL takes over.
    double rateAidBandwidthHz{1.0};              ///< Continuous slow Doppler-rate-aiding bandwidth (Hz).
    double samplingFrequencyHz{4096000.0};       ///< Sampling rate (Hz).
    int32_t numberOfSamplesPerCode{4096};        ///< Samples per code period.
    double carrierLockThreshold{0.3};            ///< Min carrier lock indicator to count as locked.
    double codeLockRatioTolerance{0.3};          ///< Max |codeLockRatio - 1.0| to count as locked.
    double lockIndicatorEmaAlpha{0.03};          ///< EMA smoothing factor for lock indicators.
    int32_t confirmDebounceBlocks{50};           ///< Good blocks needed to confirm tracking.
    int32_t confirmTimeoutBlocks{500};           ///< Blocks before abandoning an unconfirmed acquisition.
    int32_t lossDebounceBlocks{200};             ///< Bad blocks needed to declare lock lost.
    int32_t telemetryIntervalBlocks{10};         ///< Blocks between TrackingOutput publishes per channel.
};

/** @brief Tracking module output results. */
struct TrackingOutput
{
    uint32_t structVersion{STRUCT_VERSION_2};    ///< Struct version tag.
    int32_t prn{0};                              ///< Satellite PRN number.
    double carrierFreqHz{0.0};                   ///< Current carrier frequency (Hz).
    double codeFreqHz{0.0};                      ///< Current code frequency (Hz).
    double carrierErrorCycles{0.0};              ///< PLL phase error (cycles).
    double codeErrorChips{0.0};                  ///< DLL code error (chips).
    double Ie{0.0};                              ///< In-phase Early correlator.
    double Ip{0.0};                              ///< In-phase Prompt correlator.
    double Il{0.0};                              ///< In-phase Late correlator.
    double Qe{0.0};                              ///< Quadrature Early correlator.
    double Qp{0.0};                              ///< Quadrature Prompt correlator.
    double Ql{0.0};                              ///< Quadrature Late correlator.
    uint32_t channelState{0};                    ///< Channel state (0=Acquiring, 1=Confirming, 2=Tracking).
    double carrierLockIndicator{0.0};            ///< Smoothed carrier lock indicator.
    double codeLockRatio{0.0};                   ///< Smoothed code lock ratio.
    double correlatorTimeMs{0.0};                ///< Fused correlator pass duration this block (ms).
};

/** @brief Navigation decoder input parameters. */
struct NavDecoderInput
{
    uint32_t structVersion{STRUCT_VERSION_1};    ///< Struct version tag.
    uint32_t subframeSearchMask{0x1F};           ///< Bitmask of subframes to search.
};

/** @brief Navigation decoder output with ephemeris and clock data.
 *   Packed wire twin of the internal GpsEphemeris struct. Field order
 *   and scaling follow IS-GPS-200. */
struct NavDecoderOutput
{
    uint32_t structVersion{STRUCT_VERSION_1};    ///< Struct version tag.
    int32_t svId{0};                             ///< Satellite vehicle ID.
    int32_t weekNumber{0};                       ///< GPS week number.
    double tow{0.0};                             ///< Time of week (s).
    int32_t subframeId{0};                       ///< Subframe ID (1-5).
    uint32_t isValid{0};                         ///< 1 if decode valid.

    double toc{0.0};                             ///< Clock reference time (s).
    double af0{0.0};                             ///< Clock bias (s).
    double af1{0.0};                             ///< Clock drift (s/s).
    double af2{0.0};                             ///< Clock drift rate (s/s^2).
    double tgd{0.0};                             ///< Group delay differential (s).

    double toe{0.0};                             ///< Ephemeris reference time (s).
    double sqrtA{0.0};                           ///< Sqrt of semi-major axis (m^1/2).
    double e{0.0};                               ///< Orbital eccentricity.
    double i0{0.0};                              ///< Inclination at reference time (rad).
    double omega0{0.0};                          ///< Longitude of ascending node (rad).
    double omega{0.0};                           ///< Argument of perigee (rad).
    double M0{0.0};                              ///< Mean anomaly at reference time (rad).
    double deltaN{0.0};                          ///< Mean motion correction (rad/s).
    double omegaDot{0.0};                        ///< Rate of right ascension (rad/s).
    double idot{0.0};                            ///< Rate of inclination (rad/s).
    double Cuc{0.0};                             ///< Harmonic correction, cos arg of lat (rad).
    double Cus{0.0};                             ///< Harmonic correction, sin arg of lat (rad).
    double Crc{0.0};                             ///< Harmonic correction, cos orbit radius (m).
    double Crs{0.0};                             ///< Harmonic correction, sin orbit radius (m).
    double Cic{0.0};                             ///< Harmonic correction, cos inclination (rad).
    double Cis{0.0};                             ///< Harmonic correction, sin inclination (rad).
};

/** @brief PVT solver input parameters. */
struct PvtSolverInput
{
    uint32_t structVersion{STRUCT_VERSION_1};    ///< Struct version tag.
    uint32_t minSatellites{4};                   ///< Min satellites for solution.
    double maxPseudorangeErrMeters{30.0};        ///< Max converged residual (m) before rejection or exclusion.
    int32_t fixOutputIntervalBlocks{100};        ///< Blocks between PVT solve and telemetry output.
    int32_t tropoEnabled{1};                     ///< Apply Saastamoinen correction, 0 for simulated signals.
    double elevationMaskDeg{0.0};                ///< Elevation mask (deg) once a fix exists, 0 disables.
};

/** @brief PVT solver output with position and DOP. */
struct PvtSolverOutput
{
    uint32_t structVersion{STRUCT_VERSION_2};    ///< Struct version tag.
    double ecefXMeters{0.0};                     ///< ECEF X position (m).
    double ecefYMeters{0.0};                     ///< ECEF Y position (m).
    double ecefZMeters{0.0};                     ///< ECEF Z position (m).
    double latitudeDeg{0.0};                     ///< Geodetic latitude in degrees.
    double longitudeDeg{0.0};                    ///< Geodetic longitude in degrees.
    double altitudeMeters{0.0};                  ///< Altitude above WGS-84 in meters.
    double clockBiasMeters{0.0};                 ///< Receiver clock bias (m).
    double clockBiasSeconds{0.0};                ///< Receiver clock bias (s).
    double dopGDOP{0.0};                         ///< Geometric DOP. 99.9 when the DOP matrix inversion failed.
    double dopPDOP{0.0};                         ///< Position DOP. 99.9 when the DOP matrix inversion failed.
    double dopHDOP{0.0};                         ///< Horizontal DOP. 99.9 when the DOP matrix inversion failed.
    double dopVDOP{0.0};                         ///< Vertical DOP. 99.9 when the DOP matrix inversion failed.
    uint32_t isValid{0};                         ///< 1 if solution valid.
    uint32_t satellitesUsed{0};                  ///< Satellites in the accepted solution.
    double maxResidualMeters{0.0};               ///< Largest converged pseudorange residual (m).
};

/** @brief Atmospheric corrections input (Klobuchar coefficients). */
struct AtmosphericInput
{
    uint32_t structVersion{STRUCT_VERSION_1};    ///< Struct version tag.
    double alpha0{0.0};                          ///< Klobuchar alpha0 (s).
    double alpha1{0.0};                          ///< Klobuchar alpha1 (s/semi-circle).
    double alpha2{0.0};                          ///< Klobuchar alpha2 (s/semi-circle^2).
    double alpha3{0.0};                          ///< Klobuchar alpha3 (s/semi-circle^3).
    double beta0{0.0};                           ///< Klobuchar beta0 (s).
    double beta1{0.0};                           ///< Klobuchar beta1 (s/semi-circle).
    double beta2{0.0};                           ///< Klobuchar beta2 (s/semi-circle^2).
    double beta3{0.0};                           ///< Klobuchar beta3 (s/semi-circle^3).
};

/** @brief Atmospheric corrections output per satellite. */
struct AtmosphericOutput
{
    uint32_t structVersion{STRUCT_VERSION_1};    ///< Struct version tag.
    int32_t svId{0};                             ///< Satellite vehicle ID.
    double ionoDelayMeters{0.0};                 ///< Ionospheric delay (m).
    double tropoDelayMeters{0.0};                ///< Tropospheric delay (m).
    double azimuthDeg{0.0};                      ///< Satellite azimuth (deg).
    double elevationDeg{0.0};                    ///< Satellite elevation (deg).
};

/** @brief NMEA generator input parameters. */
struct NmeaGeneratorInput
{
    uint32_t structVersion{STRUCT_VERSION_1};    ///< Struct version tag.
    uint32_t enableGga{1};                       ///< Enable GGA sentences.
    uint32_t enableRmc{1};                       ///< Enable RMC sentences.
    uint32_t enableGsa{1};                       ///< Enable GSA sentences.
    uint32_t enableGsv{1};                       ///< Enable GSV sentences.
};

/** @brief NMEA generator output sentence. */
struct NmeaGeneratorOutput
{
    uint32_t structVersion{STRUCT_VERSION_1};    ///< Struct version tag.
    char sentence[256]{};                        ///< NMEA sentence string.
};

/** @brief Profiler input parameters. */
struct ProfilerInput
{
    uint32_t structVersion{STRUCT_VERSION_1};    ///< Struct version tag.
    uint32_t enabled{1};                         ///< 1 to take per-stage timing samples.
};

/** @brief Profiler output with per-stage timing. */
struct ProfilerOutput
{
    uint32_t structVersion{STRUCT_VERSION_1};    ///< Struct version tag.
    uint32_t blockIndex{0};                      ///< Processing block index.
    double timestampSec{0.0};                    ///< Block timestamp in seconds.
    double acquisitionTimeMs{0.0};               ///< Acquisition stage time (ms).
    double trackingTimeMs{0.0};                  ///< Tracking stage time (ms).
    double navDecodeTimeMs{0.0};                 ///< Nav decode stage time (ms).
    double pvtSolveTimeMs{0.0};                  ///< PVT solve stage time (ms).
    double totalTimeMs{0.0};                     ///< Total block time (ms).
    double earlyLatePromptGenTimeMs{0.0};        ///< Aggregate earlyLatePromptGen time across active channels (ms).
    double numericOscillatorTimeMs{0.0};         ///< Aggregate numericOscillator time across active channels (ms).
    double accumulatorTimeMs{0.0};          ///< Aggregate correlator-accumulation time across active channels (ms).
    double trackingMaxWorkerTimeMs{0.0};    ///< Slowest tracking worker wall-clock time this block (ms).
};

#pragma pack(pop)

}

#endif
