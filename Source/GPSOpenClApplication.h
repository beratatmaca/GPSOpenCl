#ifndef INCLUDED_GPSOPENCL_APPLICATION_H
#define INCLUDED_GPSOPENCL_APPLICATION_H

/** @file GPSOpenClApplication.h
 *  @brief Main receiver processing pipeline.
 */

#include "GPSOpenClAcquisition.h"
#include "GPSOpenClAtmosphericCorrections.h"
#include "GPSOpenClChannel.h"
#include "GPSOpenClCode.h"
#include "GPSOpenClGPUCompute.h"
#include "GPSOpenClNmeaGenerator.h"
#include "GPSOpenClNavigationDecoder.h"
#include "GPSOpenClPVTSolver.h"
#include "GPSOpenClProfiler.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClSink.h"
#include "GPSOpenClSource.h"
#include "GPSOpenClStructs.h"
#include "GPSOpenClTracking.h"

#include <memory>
#include <string>

namespace GPSOpenCl
{
/** @brief Main GPS receiver pipeline connecting all processing modules. */
class Application
{
  public:
    /** @brief Construct from configuration.
     *  @param conf Application configuration. */
    Application(Settings::Configuration conf);
    ~Application();

    /** @brief Run acquisition on all 32 PRNs.
     *  @param input IQ samples for one code period. */
    void searchForSatellites(const ComplexFloatVector &input);

    /** @brief Track all acquired satellites.
     *  @param input IQ samples for one code period. */
    void trackSatellites(const ComplexFloatVector &input);

    /** @brief Compute PVT solution from tracked satellites.
     *  @param solution Output position solution.
     *  @return True if valid solution. */
    bool computeNavigationSolution(ReceiverPvtSolution &solution);

    /** @brief Export telemetry to JSON file.
     *  @param filepath Output file path.
     *  @param solution PVT solution to export. */
    void exportTelemetryJson(const std::string &filepath, const ReceiverPvtSolution &solution);

    /** @brief Set telemetry sink.
     *  @param sink Sink implementation. */
    void setSink(std::shared_ptr<Sink> sink);

    /** @brief Set sample source.
     *  @param source Source implementation. */
    void setSource(std::shared_ptr<Source> source);

    /** @brief Get profiler reference.
     *  @return Profiler instance. */
    Profiler &getProfiler() { return m_profiler; }

    /** @brief Process one block through the full pipeline.
     *  @param input      IQ samples.
     *  @param blockIndex Current block index. */
    void processBlock(const ComplexFloatVector &input, uint32_t blockIndex);

  private:
    /** @brief Initialize all 32 satellite channels. */
    void initializeChannels();

    Acquisition *m_acquisition;              ///< Acquisition engine.
    Tracking *m_tracking;                    ///< Tracking engine.
    Settings::Configuration m_configuration; ///< Application configuration.

    Code *m_code;                            ///< C/A code generator.
    Compute *m_gpu;                          ///< GPU/CPU compute back-end.
    PVTSolver m_pvtSolver;                   ///< Position solver.
    NavigationDecoder m_navDecoder;           ///< Navigation decoder.
    NmeaGenerator m_nmeaGenerator;           ///< NMEA sentence generator.
    Channel m_channels[GPS_CA_SV_COUNT];     ///< Per-satellite channels.

    std::shared_ptr<Sink> m_sink{nullptr};   ///< Telemetry sink.
    std::shared_ptr<Source> m_source{nullptr}; ///< Sample source.
    Profiler m_profiler;                     ///< Processing time profiler.
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_APPLICATION_H
