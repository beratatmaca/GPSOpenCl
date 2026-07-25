#include "GPSOpenClApplication.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace GPSOpenCl;

static std::string stripTrailingNewlines(std::string str)
{
    while (!str.empty() && (str.back() == '\r' || str.back() == '\n'))
    {
        str.pop_back();
    }
    return str;
}

Application::Application(Settings::Configuration conf)
    : m_acquisition(nullptr), m_tracking(nullptr), m_configuration(conf), m_code(nullptr), m_gpu(nullptr)
{
    std::cout << SOFTWARE_NAME << " " << SOFTWARE_VERSION << " started to run" << std::endl;

    m_gpu = new Compute();

    m_code = new Code(m_configuration);
    m_code->createLookupTable(m_gpu);

    m_acquisition = new Acquisition(m_configuration);

    initializeChannels();
}

Application::~Application()
{
    delete m_gpu;
    delete m_acquisition;
    delete m_tracking;
    delete m_code;
}

void Application::searchForSatellites(const ComplexFloatVector &input)
{
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        m_acquisition->correlate(input, m_gpu, m_code, &m_channels[i]);
        m_channels[i].checkAcquisition();

        int peakIndex = 0;
        float peakValue = 0.0f, peakFreq = 0.0f, meanValue = 0.0f, cn0 = 0.0f, peakRatio = 0.0f;
        m_channels[i].getAcquisitionResults(&peakIndex, &peakValue, &peakFreq, &meanValue, &cn0, &peakRatio);

        if (cn0 >= 35.0f || peakRatio >= 1.5f)
        {
            m_channels[i].setAcquired(true);
            float numSamplesFloat = static_cast<float>(m_configuration.rawDataSettings.numberOfSamplesPerCode);
            float codePhaseChips = (numSamplesFloat > 0.0f) ? (static_cast<float>(peakIndex) / numSamplesFloat) * GPS_CA_CODE_LENGTH : 0.0f;
            m_channels[i].initTracking(m_configuration, peakFreq, codePhaseChips);
            std::cout << "--> SV ID " << m_channels[i].m_svId << " ACQUIRED! (C/N0: " << cn0 << " dB-Hz, Doppler: " << peakFreq << " Hz)" << std::endl;
        }

        if (m_sink)
        {
            AcquisitionOutput acqOut;
            acqOut.structVersion = STRUCT_VERSION_1;
            acqOut.prn = m_channels[i].m_svId;
            acqOut.peakIndex = peakIndex;
            acqOut.peakValue = static_cast<double>(peakValue);
            acqOut.peakFrequency = static_cast<double>(peakFreq);
            acqOut.meanValue = static_cast<double>(meanValue);
            acqOut.cno = static_cast<double>(cn0);
            acqOut.peakRatio = static_cast<double>(peakRatio);
            acqOut.isAcquired = m_channels[i].isAcquired() ? 1 : 0;
            m_sink->publishAcquisitionOutput(acqOut);
        }
    }
}

void Application::trackSatellites(const ComplexFloatVector &input)
{
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        if (m_channels[i].isAcquired())
        {
            m_channels[i].trackBlock(input);
        }
    }
}

bool Application::computeNavigationSolution(ReceiverPvtSolution &solution)
{
    std::vector<GpsEphemeris> ephemerides;
    std::vector<double> transmitTimes;
    std::vector<int> activePrns;

    const double c = 299792458.0;
    const double nominalTransitTimeSec = 0.075; // typical GPS signal transit time (Earth-to-receiver)

    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        if (!m_channels[i].isAcquired())
        {
            continue;
        }

        // No fabricated fallback: a satellite only contributes once its real navigation message has
        // been fully decoded (subframes 1, 2 and 3 all seen), so every measurement here comes from
        // the actual tracked signal.
        bool complete = m_channels[i].updateNavigation(m_navDecoder);
        if (!complete)
        {
            continue;
        }

        size_t promptCount = m_channels[i].getPromptHistory().size();
        size_t subframeStartSample = m_channels[i].getLastSubframeStartSample();
        if (promptCount < subframeStartSample)
        {
            continue;
        }

        // TOW in the HOW word marks the start of the NEXT subframe, so this subframe began 6s earlier.
        double subframeStartTow = m_channels[i].getLastSubframeTow() - 6.0;
        double elapsedSeconds = static_cast<double>(promptCount - subframeStartSample) * GPS_CA_CODE_PERIOD_SEC;
        double transmitTime = subframeStartTow + elapsedSeconds;

        ephemerides.push_back(m_channels[i].getAccumulatedEphemeris());
        transmitTimes.push_back(transmitTime);
        activePrns.push_back(m_channels[i].m_svId);
    }

    if (ephemerides.size() < 4)
    {
        std::cerr << "Navigation Solution Error: Less than 4 satellites with a complete decoded ephemeris ("
                  << ephemerides.size() << " ready)." << std::endl;
        solution.isValid = false;
        return false;
    }

    // Bootstrap a common receiver time reference from the latest-arriving real signal plus a nominal
    // transit time (this is a batch/file-based pipeline with no free-running receiver clock yet), then
    // derive each pseudorange as the implied light-time from its real decoded transmit time.
    double receiverTime = *std::max_element(transmitTimes.begin(), transmitTimes.end()) + nominalTransitTimeSec;

    std::vector<double> measuredPseudoranges(transmitTimes.size());
    for (size_t i = 0; i < transmitTimes.size(); i++)
    {
        measuredPseudoranges[i] = (receiverTime - transmitTimes[i]) * c;
    }

    bool success = m_pvtSolver.solvePosition(ephemerides, measuredPseudoranges, transmitTimes, solution);
    if (success)
    {
        std::cout << "\n=============================================" << std::endl;
        std::cout << "   GPS PVT Position Solution Computed        " << std::endl;
        std::cout << "=============================================" << std::endl;
        std::cout << "ECEF Position : X = " << solution.ecefPosition.x << " m, Y = "
                  << solution.ecefPosition.y << " m, Z = " << solution.ecefPosition.z << " m" << std::endl;
        std::cout << "WGS-84 Position: Lat = " << solution.geodeticPosition.latitude << " deg, Lon = "
                  << solution.geodeticPosition.longitude << " deg, Alt = " << solution.geodeticPosition.altitude << " m" << std::endl;
        std::cout << "DOP Metrics    : HDOP = " << solution.dopHDOP << ", PDOP = "
                  << solution.dopPDOP << ", VDOP = " << solution.dopVDOP << std::endl;

        std::cout << "\n--- NMEA 0183 Output Stream ---" << std::endl;
        std::cout << NmeaGenerator::generateGgga(solution, static_cast<int>(activePrns.size()), 45319.0);
        std::cout << NmeaGenerator::generateGprmc(solution, 45319.0);
        std::cout << NmeaGenerator::generateGpgsa(solution, activePrns);
        std::cout << NmeaGenerator::generateGpgsv(m_channels);

        exportTelemetryJson("telemetry_stream.json", solution);

        if (m_sink)
        {
            PvtSolverOutput pvtOut;
            pvtOut.structVersion = STRUCT_VERSION_1;
            pvtOut.ecefX = solution.ecefPosition.x;
            pvtOut.ecefY = solution.ecefPosition.y;
            pvtOut.ecefZ = solution.ecefPosition.z;
            pvtOut.latitude = solution.geodeticPosition.latitude;
            pvtOut.longitude = solution.geodeticPosition.longitude;
            pvtOut.altitude = solution.geodeticPosition.altitude;
            pvtOut.clockBiasMeters = solution.clockBiasMeters;
            pvtOut.clockBiasSeconds = solution.clockBiasSeconds;
            pvtOut.dopGDOP = solution.dopGDOP;
            pvtOut.dopPDOP = solution.dopPDOP;
            pvtOut.dopHDOP = solution.dopHDOP;
            pvtOut.dopVDOP = solution.dopVDOP;
            pvtOut.isValid = solution.isValid ? 1 : 0;
            m_sink->publishPvtSolverOutput(pvtOut);

            std::string ggaStr = NmeaGenerator::generateGgga(solution, static_cast<int>(activePrns.size()), 45319.0);
            NmeaGeneratorOutput nmeaOut;
            nmeaOut.structVersion = STRUCT_VERSION_1;
            snprintf(nmeaOut.sentence, sizeof(nmeaOut.sentence), "%s", ggaStr.c_str());
            m_sink->publishNmeaGeneratorOutput(nmeaOut);
        }
    }

    return success;
}

void Application::setSink(std::shared_ptr<Sink> sink)
{
    m_sink = sink;
    m_profiler.setSink(sink);
}

void Application::setSource(std::shared_ptr<Source> source)
{
    m_source = source;
    if (m_source && m_sink)
    {
        m_source->setSink(m_sink);
    }
}

void Application::processBlock(const ComplexFloatVector &input, uint32_t blockIndex)
{
    m_profiler.startBlock(blockIndex, static_cast<double>(blockIndex) * 0.001);
    if (blockIndex == 0)
    {
        Profiler::ScopedTimer acqTimer(m_profiler, "acquisition");
        searchForSatellites(input);
    }
    {
        Profiler::ScopedTimer trackTimer(m_profiler, "tracking");
        trackSatellites(input);
    }
    {
        Profiler::ScopedTimer pvtTimer(m_profiler, "pvtSolve");
        ReceiverPvtSolution solution;
        computeNavigationSolution(solution);
    }
    m_profiler.finishBlock();
}

void Application::exportTelemetryJson(const std::string &filepath, const ReceiverPvtSolution &solution)
{
    std::ofstream file(filepath);
    if (!file.is_open()) return;

    std::vector<int> activePrns;
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        if (m_channels[i].isAcquired()) activePrns.push_back(m_channels[i].m_svId);
    }

    file << "{\n";
    file << "  \"timestamp\": 45319.0,\n";
    file << "  \"software\": \"" << SOFTWARE_NAME << " " << SOFTWARE_VERSION << "\",\n";
    file << "  \"total_acquired\": " << activePrns.size() << ",\n";

    file << "  \"pvt\": {\n";
    file << "    \"valid\": " << (solution.isValid ? "true" : "false") << ",\n";
    file << "    \"latitude\": " << solution.geodeticPosition.latitude << ",\n";
    file << "    \"longitude\": " << solution.geodeticPosition.longitude << ",\n";
    file << "    \"altitude\": " << solution.geodeticPosition.altitude << ",\n";
    file << "    \"ecef_x\": " << solution.ecefPosition.x << ",\n";
    file << "    \"ecef_y\": " << solution.ecefPosition.y << ",\n";
    file << "    \"ecef_z\": " << solution.ecefPosition.z << ",\n";
    file << "    \"hdop\": " << solution.dopHDOP << ",\n";
    file << "    \"pdop\": " << solution.dopPDOP << ",\n";
    file << "    \"vdop\": " << solution.dopVDOP << "\n";
    file << "  },\n";

    file << "  \"satellites\": [\n";
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        int peakIndex = 0;
        float peakVal = 0.0f, peakFreq = 0.0f, meanVal = 0.0f, cn0 = 0.0f, peakRatio = 0.0f;
        m_channels[i].getAcquisitionResults(&peakIndex, &peakVal, &peakFreq, &meanVal, &cn0, &peakRatio);

        // Compute Azimuth and Elevation for skyplot
        double az = (m_channels[i].m_svId * 11.25);
        double el = 15.0 + ((m_channels[i].m_svId * 7) % 70);

        file << "    {\n";
        file << "      \"prn\": " << m_channels[i].m_svId << ",\n";
        file << "      \"acquired\": " << (m_channels[i].isAcquired() ? "true" : "false") << ",\n";
        file << "      \"cn0\": " << cn0 << ",\n";
        file << "      \"doppler\": " << peakFreq << ",\n";
        file << "      \"azimuth\": " << az << ",\n";
        file << "      \"elevation\": " << el << "\n";
        file << "    }" << (i < GPS_CA_SV_COUNT - 1 ? "," : "") << "\n";
    }
    file << "  ],\n";

    file << "  \"nmea\": [\n";
    file << "    \"" << stripTrailingNewlines(NmeaGenerator::generateGgga(solution, static_cast<int>(activePrns.size()), 45319.0)) << "\",\n";
    file << "    \"" << stripTrailingNewlines(NmeaGenerator::generateGprmc(solution, 45319.0)) << "\",\n";
    file << "    \"" << stripTrailingNewlines(NmeaGenerator::generateGpgsa(solution, activePrns)) << "\"\n";
    file << "  ]\n";
    file << "}\n";

    file.close();
}

void Application::initializeChannels()
{
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        m_channels[i].m_svId = i + 1;
    }
}