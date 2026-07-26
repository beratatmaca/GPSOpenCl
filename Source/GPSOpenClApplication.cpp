#include "GPSOpenClApplication.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

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
    startWorkerPool();
}

Application::~Application()
{
    stopWorkerPool();
    delete m_gpu;
    delete m_acquisition;
    delete m_tracking;
    delete m_code;
}

void Application::searchOneChannel(const ComplexFloatVector &input, int channelIndex)
{
    if (!m_channels[channelIndex].isEligibleForAcquisition())
    {
        return;
    }

    m_acquisition->correlate(input, m_gpu, m_code, &m_channels[channelIndex]);
    m_channels[channelIndex].checkAcquisition();

    int peakIndex = 0;
    float peakValue = 0.0f, peakFreq = 0.0f, meanValue = 0.0f, cn0 = 0.0f, peakRatio = 0.0f;
    m_channels[channelIndex].getAcquisitionResults(&peakIndex, &peakValue, &peakFreq, &meanValue, &cn0, &peakRatio);

    float dopplerHz = -peakFreq;

    const float acquisitionCn0ThresholdDbHz = 43.0f;
    if (cn0 >= acquisitionCn0ThresholdDbHz)
    {
        m_channels[channelIndex].setAcquired(true);
        int numberOfSamplesPerCode = m_configuration.rawDataSettings.numberOfSamplesPerCode;
        float numSamplesFloat = static_cast<float>(numberOfSamplesPerCode);
        int reflectedPeakIndex = (numberOfSamplesPerCode > 0)
            ? ((numberOfSamplesPerCode - peakIndex) % numberOfSamplesPerCode) : 0;
        float codePhaseChips = (numSamplesFloat > 0.0f) ? (static_cast<float>(reflectedPeakIndex) / numSamplesFloat) * GPS_CA_CODE_LENGTH : 0.0f;
        m_channels[channelIndex].initTracking(m_configuration, dopplerHz, codePhaseChips);
        std::cout << "--> SV ID " << m_channels[channelIndex].m_svId << " ACQUIRED! (C/N0: " << cn0
                  << " dB-Hz, Doppler: " << dopplerHz << " Hz)" << std::endl;
    }

    if (m_sink)
    {
        AcquisitionOutput acqOut;
        acqOut.structVersion = STRUCT_VERSION_1;
        acqOut.prn = m_channels[channelIndex].m_svId;
        acqOut.peakIndex = peakIndex;
        acqOut.peakValue = static_cast<double>(peakValue);
        acqOut.peakFrequency = static_cast<double>(dopplerHz);
        acqOut.meanValue = static_cast<double>(meanValue);
        acqOut.cno = static_cast<double>(cn0);
        acqOut.peakRatio = static_cast<double>(peakRatio);
        acqOut.isAcquired = m_channels[channelIndex].isAcquired() ? 1 : 0;
        m_sink->publishAcquisitionOutput(acqOut);
    }
}

void Application::searchForSatellites(const ComplexFloatVector &input)
{
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        searchOneChannel(input, i);
    }
}

void Application::trackChannelRange(const ComplexFloatVector &input, int startIdx, int endIdx)
{
    for (int i = startIdx; i < endIdx; i++)
    {
        if (m_channels[i].isTrackingLoopActive())
        {
            try
            {
                m_channels[i].trackBlock(input);
            }
            catch (const std::exception &e)
            {
                std::cerr << "Channel " << m_channels[i].m_svId << " tracking threw: " << e.what() << std::endl;
            }
        }
    }
}

void Application::workerLoop(int workerIndex)
{
    int lastSeenGeneration = 0;
    int channelsPerWorker = (GPS_CA_SV_COUNT + m_numWorkers - 1) / m_numWorkers;
    int startIdx = workerIndex * channelsPerWorker;
    int endIdx = std::min(startIdx + channelsPerWorker, GPS_CA_SV_COUNT);

    while (true)
    {
        std::unique_lock<std::mutex> lock(m_poolMutex);
        m_startCv.wait(lock, [&] { return m_shutdownWorkers || m_generation != lastSeenGeneration; });
        if (m_shutdownWorkers)
        {
            return;
        }
        lastSeenGeneration = m_generation;
        const ComplexFloatVector *input = m_currentTrackInput;
        lock.unlock();

        trackChannelRange(*input, startIdx, endIdx);

        lock.lock();
        m_pendingWorkers--;
        if (m_pendingWorkers == 0)
        {
            lock.unlock();
            m_doneCv.notify_one();
        }
    }
}

void Application::startWorkerPool()
{
    unsigned int hw = std::thread::hardware_concurrency();
    m_numWorkers = std::max(1, std::min(static_cast<int>(hw > 0 ? hw : 4), GPS_CA_SV_COUNT));
    if (m_numWorkers <= 1)
    {
        m_numWorkers = 1;
        return;
    }

    // Shrink to the number of workers that actually get a non-empty channel range - e.g. with
    // GPS_CA_SV_COUNT=32 and hardware_concurrency()=22, channelsPerWorker=ceil(32/22)=2 but
    // 22*2=44>32, so without this every worker beyond the 16th would sit idle on an empty range
    // forever, still paying the per-block wait/wake barrier cost for zero work.
    int channelsPerWorker = (GPS_CA_SV_COUNT + m_numWorkers - 1) / m_numWorkers;
    m_numWorkers = (GPS_CA_SV_COUNT + channelsPerWorker - 1) / channelsPerWorker;

    m_workers.reserve(static_cast<size_t>(m_numWorkers));
    for (int i = 0; i < m_numWorkers; i++)
    {
        m_workers.emplace_back([this, i] { workerLoop(i); });
    }
}

void Application::stopWorkerPool()
{
    if (m_workers.empty())
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(m_poolMutex);
        m_shutdownWorkers = true;
    }
    m_startCv.notify_all();

    for (auto &worker : m_workers)
    {
        if (worker.joinable())
        {
            worker.join();
        }
    }
    m_workers.clear();
}

void Application::trackSatellites(const ComplexFloatVector &input)
{
    if (m_numWorkers <= 1)
    {
        trackChannelRange(input, 0, GPS_CA_SV_COUNT);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(m_poolMutex);
        m_currentTrackInput = &input;
        m_pendingWorkers = m_numWorkers;
        m_generation++;
    }
    m_startCv.notify_all();

    std::unique_lock<std::mutex> lock(m_poolMutex);
    m_doneCv.wait(lock, [&] { return m_pendingWorkers == 0; });
}

void Application::updateChannelNavigation()
{
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        if (!m_channels[i].isTrackingConfirmed())
        {
            continue;
        }
        m_channels[i].updateNavigation(m_navDecoder);
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
        if (!m_channels[i].isTrackingConfirmed() || !m_channels[i].hasCompleteEphemeris())
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

    static int debugCallCount = 0;
    if (debugCallCount++ % 20 == 0)
    {
        std::cerr << "DEBUG PVT attempt: " << ephemerides.size() << " ready, receiverTime=" << receiverTime
                  << std::endl;
        for (size_t i = 0; i < ephemerides.size(); i++)
        {
            std::cerr << "  DEBUG PRN " << activePrns[i] << " transmitTime=" << transmitTimes[i]
                      << " pseudorange=" << measuredPseudoranges[i] << std::endl;
        }
    }

    if (m_navDecoder.hasIonosphericParams())
    {
        m_pvtSolver.setIonosphericParams(m_navDecoder.getIonosphericParams());
    }

    bool success = m_pvtSolver.solvePosition(ephemerides, measuredPseudoranges, transmitTimes, solution);
    if ((debugCallCount - 1) % 20 == 0)
    {
        std::cerr << "DEBUG solvePosition success=" << success << " ecef=(" << solution.ecefPosition.x << ","
                  << solution.ecefPosition.y << "," << solution.ecefPosition.z << ")" << std::endl;
    }
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

        std::vector<std::string> gpgsvSentences;
        if (m_nmeaGenerator.isGsvEnabled())
        {
            gpgsvSentences = NmeaGenerator::generateGpgsvSentences(m_channels, solution.ecefPosition, solution.isValid);
        }

        std::cout << "\n--- NMEA 0183 Output Stream ---" << std::endl;
        if (m_nmeaGenerator.isGgaEnabled()) std::cout << NmeaGenerator::generateGgga(solution, static_cast<int>(activePrns.size()), receiverTime);
        if (m_nmeaGenerator.isRmcEnabled()) std::cout << NmeaGenerator::generateGprmc(solution, receiverTime);
        if (m_nmeaGenerator.isGsaEnabled()) std::cout << NmeaGenerator::generateGpgsa(solution, activePrns);
        for (const std::string &gsvSentence : gpgsvSentences) std::cout << gsvSentence;

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
            for (const std::string &gsvSentence : gpgsvSentences)
            {
                NmeaGeneratorOutput nmeaOut{};
                nmeaOut.structVersion = STRUCT_VERSION_1;
                snprintf(nmeaOut.sentence, sizeof(nmeaOut.sentence), "%s", gsvSentence.c_str());
                m_sink->publishNmeaGeneratorOutput(nmeaOut);
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
    if (blockIndex == 0)
    {
        // Cold start: sweep every channel immediately so time-to-first-fix isn't held hostage to
        // the staggered schedule below.
        Profiler::ScopedTimer acqTimer(m_profiler, "acquisition");
        searchForSatellites(input);
    }
    else if (reacquisitionIntervalBlocks > 0)
    {
        // Re-searching every not-yet-acquired channel in one block turns into a multi-second stall
        // once per interval (each full Doppler search costs tens of ms, and there can be dozens of
        // still-unacquired channels). Spread the same per-channel recheck cadence across the whole
        // interval instead, one channel's search per slot, so the worst case for any single block is
        // one channel's cost, not the whole remaining constellation's.
        int32_t slotWidth = std::max<int32_t>(1, reacquisitionIntervalBlocks / GPS_CA_SV_COUNT);
        if (blockIndex % static_cast<uint32_t>(slotWidth) == 0)
        {
            int channelIndex = static_cast<int>((blockIndex / static_cast<uint32_t>(slotWidth)) %
                                                 static_cast<uint32_t>(GPS_CA_SV_COUNT));
            Profiler::ScopedTimer acqTimer(m_profiler, "acquisition");
            searchOneChannel(input, channelIndex);
        }
    }
    {
        Profiler::ScopedTimer trackTimer(m_profiler, "tracking");
        trackSatellites(input);
    }
    {
        Profiler::ScopedTimer navTimer(m_profiler, "navDecode");
        updateChannelNavigation();
    }
    int32_t fixOutputIntervalBlocks = m_configuration.pvtSolverInput.fixOutputIntervalBlocks;
    if (fixOutputIntervalBlocks <= 0 || blockIndex % static_cast<uint32_t>(fixOutputIntervalBlocks) == 0)
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
