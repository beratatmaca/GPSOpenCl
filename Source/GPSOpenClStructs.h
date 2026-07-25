#ifndef INCLUDED_GPSOPENCL_STRUCTS_H
#define INCLUDED_GPSOPENCL_STRUCTS_H

#include <cstdint>

namespace GPSOpenCl
{

#pragma pack(push, 1)

constexpr uint32_t STRUCT_VERSION_1 = 1;

// --- Source Module Structs ---
struct SourceInput
{
    uint32_t structVersion{STRUCT_VERSION_1};
    char fifoPath[256]{};
    uint32_t sampleFormat{0}; // 0 = ComplexFloat, 1 = Int8
    double samplingRate{4096000.0};
};

struct SourceOutput
{
    uint32_t structVersion{STRUCT_VERSION_1};
    uint32_t blockIndex{0};
    double timestamp{0.0};
    uint32_t fifoUnderrunCount{0};
    uint32_t fifoOverrunCount{0};
};

// --- Acquisition Module Structs ---
struct AcquisitionInput
{
    uint32_t structVersion{STRUCT_VERSION_1};
    int32_t acquisitionDopplerMinimum{-4000};
    int32_t acquisitionDopplerMaximum{4000};
    int32_t acquisitionDopplerSearchRange{500};
    double samplingFrequency{4096000.0};
    int32_t numberOfSamplesPerCode{4096};
};

struct AcquisitionOutput
{
    uint32_t structVersion{STRUCT_VERSION_1};
    int32_t prn{0};
    int32_t peakIndex{0};
    double peakValue{0.0};
    double peakFrequency{0.0};
    double meanValue{0.0};
    double cno{0.0};
    double peakRatio{0.0};
    uint32_t isAcquired{0};
};

// --- Tracking Module Structs ---
struct TrackingInput
{
    uint32_t structVersion{STRUCT_VERSION_1};
    double pllBandwidthHz{25.0};
    double dllBandwidthHz{2.0};
    double samplingFrequency{4096000.0};
    int32_t numberOfSamplesPerCode{4096};
};

struct TrackingOutput
{
    uint32_t structVersion{STRUCT_VERSION_1};
    int32_t prn{0};
    double carrierFreqHz{0.0};
    double codeFreqHz{0.0};
    double carrierError{0.0};
    double codeError{0.0};
    double Ie{0.0};
    double Ip{0.0};
    double Il{0.0};
    double Qe{0.0};
    double Qp{0.0};
    double Ql{0.0};
};

// --- Navigation Decoder Module Structs ---
struct NavDecoderInput
{
    uint32_t structVersion{STRUCT_VERSION_1};
    uint32_t subframeSearchMask{0x1F};
};

struct NavDecoderOutput
{
    uint32_t structVersion{STRUCT_VERSION_1};
    int32_t svId{0};
    int32_t weekNumber{0};
    double tow{0.0};
    int32_t subframeId{0};
    uint32_t isValid{0};

    // Clock correction parameters
    double toc{0.0};
    double af0{0.0};
    double af1{0.0};
    double af2{0.0};

    // Ephemeris orbit parameters
    double toe{0.0};
    double sqrtA{0.0};
    double e{0.0};
    double i0{0.0};
    double omega0{0.0};
    double omega{0.0};
    double M0{0.0};
    double deltaN{0.0};
    double omegaDot{0.0};
    double idot{0.0};
    double Cuc{0.0};
    double Cus{0.0};
    double Crc{0.0};
    double Crs{0.0};
    double Cic{0.0};
    double Cis{0.0};
};

// --- PVT Solver Module Structs ---
struct PvtSolverInput
{
    uint32_t structVersion{STRUCT_VERSION_1};
    uint32_t minSatellites{4};
    double maxPseudorangeErrMeters{100.0};
};

struct PvtSolverOutput
{
    uint32_t structVersion{STRUCT_VERSION_1};
    double ecefX{0.0};
    double ecefY{0.0};
    double ecefZ{0.0};
    double latitude{0.0};
    double longitude{0.0};
    double altitude{0.0};
    double clockBiasMeters{0.0};
    double clockBiasSeconds{0.0};
    double dopGDOP{0.0};
    double dopPDOP{0.0};
    double dopHDOP{0.0};
    double dopVDOP{0.0};
    uint32_t isValid{0};
};

// --- Atmospheric Corrections Module Structs ---
struct AtmosphericInput
{
    uint32_t structVersion{STRUCT_VERSION_1};
    double alpha0{0.0};
    double alpha1{0.0};
    double alpha2{0.0};
    double alpha3{0.0};
    double beta0{0.0};
    double beta1{0.0};
    double beta2{0.0};
    double beta3{0.0};
};

struct AtmosphericOutput
{
    uint32_t structVersion{STRUCT_VERSION_1};
    int32_t svId{0};
    double ionoDelayMeters{0.0};
    double tropoDelayMeters{0.0};
    double azimuthDeg{0.0};
    double elevationDeg{0.0};
};

// --- NMEA Generator Module Structs ---
struct NmeaGeneratorInput
{
    uint32_t structVersion{STRUCT_VERSION_1};
    uint32_t enableGga{1};
    uint32_t enableRmc{1};
    uint32_t enableGsa{1};
    uint32_t enableGsv{1};
};

struct NmeaGeneratorOutput
{
    uint32_t structVersion{STRUCT_VERSION_1};
    char sentence[256]{};
};

// --- Profiler Module Struct ---
struct ProfilerOutput
{
    uint32_t structVersion{STRUCT_VERSION_1};
    uint32_t blockIndex{0};
    double timestamp{0.0};
    double acquisitionTimeMs{0.0};
    double trackingTimeMs{0.0};
    double navDecodeTimeMs{0.0};
    double pvtSolveTimeMs{0.0};
    double totalTimeMs{0.0};
};

#pragma pack(pop)

} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_STRUCTS_H
