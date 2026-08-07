#include "GPSOpenClTelemetryExporter.h"

#include "GPSOpenClAtmosphericCorrections.h"
#include "GPSOpenClCommon.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <utility>

using namespace GPSOpenCl;

namespace
{
std::string stripTrailingNewlines(std::string str)
{
    while (!str.empty() && (str.back() == '\r' || str.back() == '\n'))
    {
        str.pop_back();
    }
    return str;
}
}

TelemetryExporter::TelemetryExporter()
{
    m_writerThread = std::thread([this] { writerLoop(); });
}

TelemetryExporter::~TelemetryExporter()
{
    m_queue.finish();
    if (m_writerThread.joinable())
    {
        m_writerThread.join();
    }
}

void TelemetryExporter::pushConsole(std::string text)
{
    AsyncOutputJob job;
    job.isConsole = true;
    job.content = std::move(text);
    m_queue.tryPush(std::move(job));
}

void TelemetryExporter::pushJsonFile(const std::string &filepath, std::unique_ptr<TelemetrySnapshot> snapshot)
{
    AsyncOutputJob job;
    job.isConsole = false;
    job.filePath = filepath;
    job.telemetry = std::move(snapshot);
    m_queue.tryPush(std::move(job));
}

void TelemetryExporter::writerLoop()
{
    AsyncOutputJob job;
    while (m_queue.pop(job))
    {
        try
        {
            if (job.telemetry)
            {
                job.content = formatTelemetryJson(*job.telemetry);
                job.telemetry.reset();
            }
            if (job.isConsole)
            {
                std::cout << job.content;
            }
            else
            {
                std::ofstream file(job.filePath);
                if (file.is_open())
                {
                    file << job.content;
                    file.close();
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Telemetry exporter: job dropped after exception: " << e.what() << '\n';
        }
    }
}

std::string TelemetryExporter::formatTelemetryJson(const TelemetrySnapshot &snapshot)
{
    const ReceiverPvtSolution &solution = snapshot.solution;
    std::ostringstream file;

    const auto json = [](double value) { return std::isfinite(value) ? value : 0.0; };

    file << "{\n";
    file << "  \"timestamp\": " << json(snapshot.utcTimeSec) << ",\n";
    file << R"(  "software": ")" << SOFTWARE_NAME << " " << SOFTWARE_VERSION << "\",\n";
    file << "  \"total_acquired\": " << snapshot.activePrns.size() << ",\n";

    file << "  \"pvt\": {\n";
    file << "    \"valid\": " << (solution.isValid ? "true" : "false") << ",\n";
    file << "    \"latitude\": " << json(solution.geodeticPosition.latitudeDeg) << ",\n";
    file << "    \"longitude\": " << json(solution.geodeticPosition.longitudeDeg) << ",\n";
    file << "    \"altitude\": " << json(solution.geodeticPosition.altitudeMeters) << ",\n";
    file << "    \"ecef_x\": " << json(solution.ecefPosition.x) << ",\n";
    file << "    \"ecef_y\": " << json(solution.ecefPosition.y) << ",\n";
    file << "    \"ecef_z\": " << json(solution.ecefPosition.z) << ",\n";
    file << "    \"hdop\": " << json(solution.dopHDOP) << ",\n";
    file << "    \"pdop\": " << json(solution.dopPDOP) << ",\n";
    file << "    \"vdop\": " << json(solution.dopVDOP) << "\n";
    file << "  },\n";

    file << "  \"satellites\": [\n";
    for (size_t i = 0; i < snapshot.satellites.size(); i++)
    {
        const SatelliteTelemetry &sat = snapshot.satellites[i];

        bool hasPosition = false;
        double az = 0.0;
        double el = 0.0;
        if (sat.computeOrbit)
        {
            const SatelliteOrbit orbit = PVTSolver::computeSatelliteOrbit(sat.ephemeris, sat.transmitTime);
            AtmosphericCorrections::computeAzimuthElevation(solution.ecefPosition, orbit.position, az, el);
            hasPosition = true;
        }

        file << "    {\n";
        file << "      \"prn\": " << sat.prn << ",\n";
        file << "      \"acquired\": " << (sat.acquired ? "true" : "false") << ",\n";
        file << "      \"cn0\": " << json(sat.cn0) << ",\n";
        file << "      \"doppler\": " << json(sat.doppler) << ",\n";
        file << "      \"has_position\": " << (hasPosition ? "true" : "false") << ",\n";
        file << "      \"azimuth\": " << json(az) << ",\n";
        file << "      \"elevation\": " << json(el) << "\n";
        file << "    }" << (i + 1 < snapshot.satellites.size() ? "," : "") << "\n";
    }
    file << "  ],\n";

    file << "  \"nmea\": [\n";
    file << "    \"" << stripTrailingNewlines(snapshot.ggaSentence) << "\",\n";
    file << "    \"" << stripTrailingNewlines(snapshot.rmcSentence) << "\",\n";
    file << "    \"" << stripTrailingNewlines(snapshot.gsaSentence) << "\"\n";
    file << "  ]\n";
    file << "}\n";

    return file.str();
}
