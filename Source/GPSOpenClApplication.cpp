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
    : m_acquisition(nullptr), m_tracking(nullptr), m_configuration(conf), m_code(nullptr), m_gpu(nullptr),
      m_pvtSolver(conf.pvtSolverInput), m_navDecoder(conf.navDecoderInput), m_nmeaGenerator(conf.nmeaGeneratorInput)
{
    std::cout << SOFTWARE_NAME << " " << SOFTWARE_VERSION << " started to run" << std::endl;

    m_gpu = new Compute();

    m_code = new Code(m_configuration);
    m_code->createLookupTable(m_gpu);

    m_acquisition = new Acquisition(m_configuration);

    m_pvtSolver.setIonosphericParams(conf.atmosphericInput);

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
        if (!m_channels[i].isEligibleForAcquisition())
        {
            continue;
        }

        m_acquisition->correlate(input, m_gpu, m_code, &m_channels[i]);
        m_channels[i].checkAcquisition();

        int peakIndex = 0;
        float peakValue = 0.0f, peakFreq = 0.0f, meanValue = 0.0f, cn0 = 0.0f, peakRatio = 0.0f;
        m_channels[i].getAcquisitionResults(&peakIndex, &peakValue, &peakFreq, &meanValue, &cn0, &peakRatio);

        float dopplerHz = -peakFreq;

        const float acquisitionCn0ThresholdDbHz = 43.0f;
        if (cn0 >= acquisitionCn0ThresholdDbHz)
        {
            m_channels[i].setAcquired(true);
            int numberOfSamplesPerCode = m_configuration.rawDataSettings.numberOfSamplesPerCode;
            float numSamplesFloat = static_cast<float>(numberOfSamplesPerCode);
            int reflectedPeakIndex = (numberOfSamplesPerCode > 0)
                ? ((numberOfSamplesPerCode - peakIndex) % numberOfSamplesPerCode) : 0;
            float codePhaseChips = (numSamplesFloat > 0.0f) ? (static_cast<float>(reflectedPeakIndex) / numSamplesFloat) * GPS_CA_CODE_LENGTH : 0.0f;
            m_channels[i].initTracking(m_configuration, dopplerHz, codePhaseChips);
            std::cout << "--> SV ID " << m_channels[i].m_svId << " ACQUIRED! (C/N0: " << cn0 << " dB-Hz, Doppler: " << dopplerHz << " Hz)" << std::endl;
        }

        if (m_sink)
        {
            AcquisitionOutput acqOut;
            acqOut.structVersion = STRUCT_VERSION_1;
            acqOut.prn = m_channels[i].m_svId;
            acqOut.peakIndex = peakIndex;
            acqOut.peakValue = static_cast<double>(peakValue);
            acqOut.peakFrequency = static_cast<double>(dopplerHz);
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
        if (m_channels[i].isTrackingLoopActive())
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
    const double nominalTransitTimeSec = 0.075;

    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        if (!m_channels[i].isTrackingConfirmed())
        {
            continue;
        }




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




    double receiverTime = *std::max_element(transmitTimes.begin(), transmitTimes.end()) + nominalTransitTimeSec;

    std::vector<double> measuredPseudoranges(transmitTimes.size());
    for (size_t i = 0; i < transmitTimes.size(); i++)
    {
        measuredPseudoranges[i] = (receiverTime - transmitTimes[i]) * c;
    }

    if (m_navDecoder.hasIonosphericParams())
    {
        m_pvtSolver.setIonosphericParams(m_navDecoder.getIonosphericParams());
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
        if (m_nmeaGenerator.isGgaEnabled()) std::cout << NmeaGenerator::generateGgga(solution, static_cast<int>(activePrns.size()), receiverTime);
        if (m_nmeaGenerator.isRmcEnabled()) std::cout << NmeaGenerator::generateGprmc(solution, receiverTime);
        if (m_nmeaGenerator.isGsaEnabled()) std::cout << NmeaGenerator::generateGpgsa(solution, activePrns);
        if (m_nmeaGenerator.isGsvEnabled()) std::cout << NmeaGenerator::generateGpgsv(m_channels, solution.ecefPosition, solution.isValid);

        exportTelemetryJson("telemetry_stream.json", solution, receiverTime);

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

            if (m_nmeaGenerator.isGgaEnabled())
            {
                NmeaGeneratorOutput nmeaOut = NmeaGenerator::generateGggaOutput(solution, static_cast<int>(activePrns.size()), receiverTime);
                m_sink->publishNmeaGeneratorOutput(nmeaOut);
            }
            if (m_nmeaGenerator.isRmcEnabled())
            {
                NmeaGeneratorOutput nmeaOut = NmeaGenerator::generateGprmcOutput(solution, receiverTime);
                m_sink->publishNmeaGeneratorOutput(nmeaOut);
            }
            if (m_nmeaGenerator.isGsaEnabled())
            {
                NmeaGeneratorOutput nmeaOut = NmeaGenerator::generateGpgsaOutput(solution, activePrns);
                m_sink->publishNmeaGeneratorOutput(nmeaOut);
            }
            if (m_nmeaGenerator.isGsvEnabled())
            {
                for (const NmeaGeneratorOutput &nmeaOut :
                     NmeaGenerator::generateGpgsvOutput(m_channels, solution.ecefPosition, solution.isValid))
                {
                    m_sink->publishNmeaGeneratorOutput(nmeaOut);
                }
            }
        }
    }

    return success;
}

void Application::setSink(std::shared_ptr<Sink> sink)
{
    m_sink = sink;
    m_profiler.setSink(sink);
    m_navDecoder.setSink(sink);
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        m_channels[i].setSink(sink);
    }
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
    int32_t reacquisitionIntervalBlocks = m_configuration.acquisitionInput.reacquisitionIntervalBlocks;
    if (blockIndex == 0 || (reacquisitionIntervalBlocks > 0 && blockIndex % reacquisitionIntervalBlocks == 0))
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

void Application::exportTelemetryJson(const std::string &filepath, const ReceiverPvtSolution &solution, double utcTimeSec)
{
    std::ofstream file(filepath);
    if (!file.is_open()) return;

    std::vector<int> activePrns;
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        if (m_channels[i].isAcquired()) activePrns.push_back(m_channels[i].m_svId);
    }

    file << "{\n";
    file << "  \"timestamp\": " << utcTimeSec << ",\n";
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

        bool hasPosition = false;
        double az = 0.0;
        double el = 0.0;

        if (solution.isValid && m_channels[i].hasCompleteEphemeris())
        {
            size_t promptCount = m_channels[i].getPromptHistory().size();
            size_t subframeStartSample = m_channels[i].getLastSubframeStartSample();
            if (promptCount >= subframeStartSample)
            {
                double subframeStartTow = m_channels[i].getLastSubframeTow() - 6.0;
                double elapsedSeconds = static_cast<double>(promptCount - subframeStartSample) * GPS_CA_CODE_PERIOD_SEC;
                double transmitTime = subframeStartTow + elapsedSeconds;

                SatelliteOrbit orbit =
                    PVTSolver::computeSatelliteOrbit(m_channels[i].getAccumulatedEphemeris(), transmitTime);
                AtmosphericCorrections::computeAzimuthElevation(solution.ecefPosition, orbit.position, az, el);
                hasPosition = true;
            }
        }

        file << "    {\n";
        file << "      \"prn\": " << m_channels[i].m_svId << ",\n";
        file << "      \"acquired\": " << (m_channels[i].isAcquired() ? "true" : "false") << ",\n";
        file << "      \"cn0\": " << cn0 << ",\n";
        file << "      \"doppler\": " << -peakFreq << ",\n";
        file << "      \"has_position\": " << (hasPosition ? "true" : "false") << ",\n";
        file << "      \"azimuth\": " << az << ",\n";
        file << "      \"elevation\": " << el << "\n";
        file << "    }" << (i < GPS_CA_SV_COUNT - 1 ? "," : "") << "\n";
    }
    file << "  ],\n";

    file << "  \"nmea\": [\n";
    file << "    \"" << stripTrailingNewlines(NmeaGenerator::generateGgga(solution, static_cast<int>(activePrns.size()), utcTimeSec)) << "\",\n";
    file << "    \"" << stripTrailingNewlines(NmeaGenerator::generateGprmc(solution, utcTimeSec)) << "\",\n";
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
