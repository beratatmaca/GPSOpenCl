#ifndef INCLUDED_GPSOPENCL_TELEMETRYEXPORTER_HPP
#define INCLUDED_GPSOPENCL_TELEMETRYEXPORTER_HPP

/** @file GPSOpenClTelemetryExporter.hpp
 *  @brief Background writer for console text and telemetry JSON.
 */

#include "Common/GPSOpenClBoundedQueue.hpp"
#include "NavDecode/GPSOpenClNavigationDecoder.hpp"
#include "Pvt/GPSOpenClPVTSolver.hpp"

#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace GPSOpenCl
{
/** @brief Writes console output and telemetry JSON off the hot path.
 *   Internal pipeline helper, not a telemetry module. The consumer
 *   thread hands work over a dropping queue. A background thread does
 *   the slow formatting and I O. The consumer never blocks on stdout
 *   or disk. */
class TelemetryExporter
{
  public:
    /** @brief Per satellite state captured for deferred JSON formatting. */
    struct SatelliteTelemetry
    {
        int prn{0};                  ///< Satellite PRN.
        bool acquired{false};        ///< True if the channel is acquired.
        float cn0{0.0f};             ///< Acquisition C/N0 estimate in dB-Hz.
        float doppler{0.0f};         ///< Acquired Doppler in Hz.
        bool computeOrbit{false};    ///< True if ephemeris and transmitTime allow az el.
        GpsEphemeris ephemeris{};    ///< Decoded ephemeris for orbit computation.
        double transmitTime{0.0};    ///< Transmit time for orbit computation in seconds.
    };

    /** @brief Snapshot of everything the telemetry JSON needs. Captured
     *   cheaply on the consumer thread. Orbit math and formatting run
     *   on the writer thread. */
    struct TelemetrySnapshot
    {
        ReceiverPvtSolution solution{};                ///< PVT solution.
        double utcTimeSec{0.0};                        ///< Receiver GPS time of week in seconds.
        std::vector<int> activePrns;                   ///< PRNs of acquired channels.
        std::vector<SatelliteTelemetry> satellites;    ///< Per satellite state.
        std::string ggaSentence;                       ///< Pre generated GGA sentence.
        std::string rmcSentence;                       ///< Pre generated RMC sentence.
        std::string gsaSentence;                       ///< Pre generated GSA sentence.
    };

    TelemetryExporter();
    ~TelemetryExporter();
    TelemetryExporter(const TelemetryExporter &) = delete;
    TelemetryExporter &operator=(const TelemetryExporter &) = delete;
    TelemetryExporter(TelemetryExporter &&) = delete;
    TelemetryExporter &operator=(TelemetryExporter &&) = delete;

    /** @brief Queue text for stdout. Drops when the queue is full.
     *  @param text Console text. */
    void pushConsole(std::string text);

    /** @brief Queue a snapshot for JSON formatting and file write.
     *  @param filepath Output file path.
     *  @param snapshot Captured telemetry state. */
    void pushJsonFile(const std::string &filepath, std::unique_ptr<TelemetrySnapshot> snapshot);

    /** @brief Format a telemetry snapshot as JSON. Computes satellite
     *   orbits and azimuth elevation. Non finite values become 0.
     *  @param snapshot Captured telemetry state.
     *  @return JSON document text. */
    static std::string formatTelemetryJson(const TelemetrySnapshot &snapshot);

  private:
    /** @brief One pending write. Either a console print or a file overwrite. */
    struct AsyncOutputJob
    {
        bool isConsole{false};                           ///< True writes stdout. False overwrites filePath.
        std::string filePath;                            ///< Target file path, unused for console jobs.
        std::string content;                             ///< Text to write.
        std::unique_ptr<TelemetrySnapshot> telemetry;    ///< Deferred snapshot, or null.
    };

    /** @brief Writer thread body. Drains the queue and writes. */
    void writerLoop();

    BoundedQueue<AsyncOutputJob> m_queue{32};    ///< Handoff queue for writes.
    std::thread m_writerThread;                  ///< Background writer thread.
};
}

#endif
