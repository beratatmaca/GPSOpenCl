#include "GPSOpenClApplication.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
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

bool computeElapsedSecondsSincePromptStart(size_t promptCount, size_t startSample, double &elapsedSecondsOut)
{
    if (promptCount < startSample)
    {
        return false;
    }
    elapsedSecondsOut = static_cast<double>(promptCount - startSample) * GPS_CA_CODE_PERIOD_SEC;
    return true;
}
}

Application::Application(const Settings::Configuration &conf)
    : m_acquisition(nullptr),
      m_tracking(nullptr),
      m_configuration(conf),
      m_code(nullptr),
      m_gpu(std::make_unique<Compute>()),
      m_pvtSolver(conf.pvtSolverInput),
      m_navDecoder(conf.navDecoderInput),
      m_nmeaGenerator(conf.nmeaGeneratorInput)
{
    std::cout << SOFTWARE_NAME << " " << SOFTWARE_VERSION << " started to run" << '\n';

    m_code = std::make_unique<Code>(m_configuration);
    m_code->createLookupTable(m_gpu.get());

    m_acquisition = std::make_unique<Acquisition>(m_configuration);

    m_pvtSolver.setIonosphericParams(conf.atmosphericInput);

    initializeChannels();
    startWorkerPool();
    m_acquisitionThread = std::thread([this] { acquisitionWorkerLoop(); });
    m_outputWriterThread = std::thread([this] { outputWriterLoop(); });
}

Application::~Application()
{
    m_acquisitionJobQueue.finish();
    if (m_acquisitionThread.joinable())
    {
        m_acquisitionThread.join();
    }
    m_outputQueue.finish();
    if (m_outputWriterThread.joinable())
    {
        m_outputWriterThread.join();
    }
    stopWorkerPool();
    delete m_tracking;
}

void Application::searchOneChannel(const ComplexFloatVector &input, int channelIndex)
{
    if (!m_channels[channelIndex].isEligibleForAcquisition())
    {
        return;
    }

    m_acquisition->correlate(input, m_gpu.get(), m_code.get(), &m_channels[channelIndex]);
    finalizeAcquisition(channelIndex);
}

void Application::finalizeAcquisition(int channelIndex)
{
    Channel &channel = m_channels[channelIndex];
    channel.checkAcquisition();

    int peakIndex = 0;
    float peakValue = 0.0f;
    float peakFreq = 0.0f;
    float meanValue = 0.0f;
    float cn0 = 0.0f;
    float peakRatio = 0.0f;
    channel.getAcquisitionResults(&peakIndex, &peakValue, &peakFreq, &meanValue, &cn0, &peakRatio);

    const float dopplerHz = -peakFreq;

    const float acquisitionCn0ThresholdDbHz = 43.0f;
    if (cn0 >= acquisitionCn0ThresholdDbHz)
    {
        channel.setAcquired(true);
        const int numberOfSamplesPerCode = m_configuration.rawDataSettings.numberOfSamplesPerCode;
        auto numSamplesFloat = static_cast<float>(numberOfSamplesPerCode);
        const int reflectedPeakIndex =
            (numberOfSamplesPerCode > 0) ? ((numberOfSamplesPerCode - peakIndex) % numberOfSamplesPerCode) : 0;
        const float codePhaseChips = (numSamplesFloat > 0.0f)
            ? (static_cast<float>(reflectedPeakIndex) / numSamplesFloat) * GPS_CA_CODE_LENGTH
            : 0.0f;
        channel.initTracking(m_configuration, dopplerHz, codePhaseChips);
        std::cout << "--> SV ID " << channel.m_svId << " ACQUIRED! (C/N0: " << cn0 << " dB-Hz, Doppler: " << dopplerHz
                  << " Hz)" << '\n';
    }

    if (m_sink)
    {
        AcquisitionOutput acqOut;
        acqOut.structVersion = STRUCT_VERSION_1;
        acqOut.prn = channel.m_svId;
        acqOut.peakIndex = peakIndex;
        acqOut.peakValue = static_cast<double>(peakValue);
        acqOut.peakFrequency = static_cast<double>(dopplerHz);
        acqOut.meanValue = static_cast<double>(meanValue);
        acqOut.cno = static_cast<double>(cn0);
        acqOut.peakRatio = static_cast<double>(peakRatio);
        acqOut.isAcquired = channel.isAcquired() ? 1 : 0;
        m_sink->publishAcquisitionOutput(acqOut);
    }
}

void Application::acquisitionWorkerLoop()
{
    AcquisitionJob job;
    while (m_acquisitionJobQueue.pop(job))
    {
        auto start = std::chrono::high_resolution_clock::now();
        m_acquisition->correlate(job.input, m_gpu.get(), m_code.get(), &m_channels[job.channelIndex]);
        auto end = std::chrono::high_resolution_clock::now();

        AcquisitionResult result;
        result.channelIndex = job.channelIndex;
        result.correlateMs = std::chrono::duration<double, std::milli>(end - start).count();
        m_acquisitionResultQueue.tryPush(result);
    }
}

void Application::outputWriterLoop()
{
    AsyncOutputJob job;
    while (m_outputQueue.pop(job))
    {
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
                std::cerr << "Channel " << m_channels[i].m_svId << " tracking threw: " << e.what() << '\n';
            }
        }
    }
}

void Application::workerLoop(int workerIndex)
{
    int lastSeenGeneration = 0;
    const int channelsPerWorker = (GPS_CA_SV_COUNT + m_numWorkers - 1) / m_numWorkers;
    const int startIdx = workerIndex * channelsPerWorker;
    const int endIdx = std::min(startIdx + channelsPerWorker, GPS_CA_SV_COUNT);

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

        auto workStart = std::chrono::high_resolution_clock::now();
        trackChannelRange(*input, startIdx, endIdx);
        auto workEnd = std::chrono::high_resolution_clock::now();
        m_workerDurationMs[static_cast<size_t>(workerIndex)] =
            std::chrono::duration<double, std::milli>(workEnd - workStart).count();

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
    const unsigned int hw = std::thread::hardware_concurrency();
    m_numWorkers = std::max(1, std::min(static_cast<int>(hw > 0 ? hw : 4), GPS_CA_SV_COUNT));
    if (m_numWorkers <= 1)
    {
        m_numWorkers = 1;
        m_workerDurationMs.assign(1, 0.0);
        return;
    }

    // Shrink to the number of workers that actually get a non-empty channel range - e.g. with
    // GPS_CA_SV_COUNT=32 and hardware_concurrency()=22, channelsPerWorker=ceil(32/22)=2 but
    // 22*2=44>32, so without this every worker beyond the 16th would sit idle on an empty range
    // forever, still paying the per-block wait/wake barrier cost for zero work.
    const int channelsPerWorker = (GPS_CA_SV_COUNT + m_numWorkers - 1) / m_numWorkers;
    m_numWorkers = (GPS_CA_SV_COUNT + channelsPerWorker - 1) / channelsPerWorker;

    m_workerDurationMs.assign(static_cast<size_t>(m_numWorkers), 0.0);
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
        const std::lock_guard<std::mutex> lock(m_poolMutex);
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
        auto workStart = std::chrono::high_resolution_clock::now();
        trackChannelRange(input, 0, GPS_CA_SV_COUNT);
        auto workEnd = std::chrono::high_resolution_clock::now();
        if (!m_workerDurationMs.empty())
        {
            m_workerDurationMs[0] = std::chrono::duration<double, std::milli>(workEnd - workStart).count();
        }
        return;
    }

    {
        const std::lock_guard<std::mutex> lock(m_poolMutex);
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
    for (auto &channel : m_channels)
    {
        if (!channel.isTrackingConfirmed())
        {
            continue;
        }
        channel.updateNavigation(m_navDecoder);
    }
}

std::vector<Application::ChannelDiagnostic> Application::getChannelDiagnostics() const
{
    std::vector<ChannelDiagnostic> diagnostics;
    for (const auto &channel : m_channels)
    {
        if (!channel.isTrackingConfirmed() || channel.getBitSyncPhase() < 0 ||
            channel.getLastSubframeStartSample() == 0)
        {
            continue;
        }

        const size_t promptCount = channel.getPromptHistory().size();
        if (promptCount == 0)
        {
            continue;
        }

        ChannelDiagnostic diag;
        diag.svId = channel.m_svId;
        diag.bitSyncPhase = channel.getBitSyncPhase();
        diag.subframeStartTow = channel.getLastSubframeTow() - 6.0;
        diag.subframeStartSample = channel.getLastSubframeStartSample();
        diag.codePhaseAtSubframeStart = channel.getCodePhaseAtSample(diag.subframeStartSample);
        if (!computeElapsedSecondsSincePromptStart(promptCount, diag.subframeStartSample, diag.elapsedSeconds))
        {
            continue;
        }
        diag.candidateNowTow = diag.subframeStartTow + diag.elapsedSeconds;
        diag.codePhaseNow = channel.getCodePhaseAtSample(promptCount - 1);
        diagnostics.push_back(diag);
    }
    return diagnostics;
}

bool Application::computeNavigationSolution(ReceiverPvtSolution &solution)
{
    std::vector<GpsEphemeris> ephemerides;
    std::vector<double> transmitTimes;
    std::vector<int> activePrns;

    const double c = 299792458.0;

    for (auto &channel : m_channels)
    {
        if (!channel.isTrackingConfirmed() || !channel.hasCompleteEphemeris())
        {
            continue;
        }

        const size_t promptCount = channel.getPromptHistory().size();
        const size_t subframeStartSample = channel.getLastSubframeStartSample();
        double elapsedSeconds = 0.0;
        if (!computeElapsedSecondsSincePromptStart(promptCount, subframeStartSample, elapsedSeconds))
        {
            continue;
        }

        const double subframeStartTow = channel.getLastSubframeTow() - 6.0;

        const double driftChips = static_cast<double>(channel.getCumulativeDriftChipsAtSample(promptCount - 1)) -
            static_cast<double>(channel.getCumulativeDriftChipsAtSample(subframeStartSample));
        static int debugDriftCount = 0;
        if (debugDriftCount++ % 200 == 0)
        {
            std::cerr << "  DEBUG drift svId=" << channel.m_svId << " promptCount=" << promptCount
                      << " subframeStartSample=" << subframeStartSample << " driftChips=" << driftChips << '\n';
        }

        const double transmitTime = subframeStartTow + elapsedSeconds + (driftChips / GPS_CA_CODE_FREQUENCY_HZ);

        ephemerides.push_back(channel.getAccumulatedEphemeris());
        transmitTimes.push_back(transmitTime);
        activePrns.push_back(channel.m_svId);
    }

    if (ephemerides.size() < 4)
    {
        static int insufficientSatCount = 0;
        if (insufficientSatCount++ % 20 == 0)
        {
            std::cerr << "Navigation Solution Error: Less than 4 satellites with a complete decoded ephemeris ("
                      << ephemerides.size() << " ready)." << '\n';
        }
        solution.isValid = false;
        m_lastPseudorangeSamples.clear();
        return false;
    }

    const EcefPosition referenceEcef = m_pvtSolver.getReferenceEcef();
    const double receiverTime = PVTSolver::computeReceiverTime(ephemerides, transmitTimes, referenceEcef);

    std::vector<double> measuredPseudoranges(transmitTimes.size());
    for (size_t i = 0; i < transmitTimes.size(); i++)
    {
        measuredPseudoranges[i] = (receiverTime - transmitTimes[i]) * c;
    }

    m_lastPseudorangeSamples.clear();
    for (size_t i = 0; i < activePrns.size(); i++)
    {
        m_lastPseudorangeSamples.push_back({activePrns[i], transmitTimes[i], measuredPseudoranges[i]});
    }

    static int debugCallCount = 0;
    if (debugCallCount++ % 20 == 0)
    {
        std::cerr << std::setprecision(15) << "DEBUG PVT attempt: " << ephemerides.size()
                  << " ready, receiverTime=" << receiverTime << '\n';
        for (size_t i = 0; i < ephemerides.size(); i++)
        {
            std::cerr << "  DEBUG PRN " << activePrns[i] << " transmitTime=" << transmitTimes[i]
                      << " pseudorange=" << measuredPseudoranges[i] << '\n';
        }
        std::cerr << std::setprecision(6);
    }

    if (m_navDecoder.hasIonosphericParams())
    {
        m_pvtSolver.setIonosphericParams(m_navDecoder.getIonosphericParams());
    }

    const bool success = m_pvtSolver.solvePosition(ephemerides, measuredPseudoranges, transmitTimes, solution);
    if ((debugCallCount - 1) % 20 == 0)
    {
        std::cerr << "DEBUG solvePosition success=" << success << " ecef=(" << solution.ecefPosition.x << ","
                  << solution.ecefPosition.y << "," << solution.ecefPosition.z << ")" << '\n';
    }
    if (success)
    {
        std::ostringstream consoleOut;
        consoleOut << "\n=============================================" << '\n';
        consoleOut << "   GPS PVT Position Solution Computed        " << '\n';
        consoleOut << "=============================================" << '\n';
        consoleOut << "ECEF Position : X = " << solution.ecefPosition.x << " m, Y = " << solution.ecefPosition.y
                   << " m, Z = " << solution.ecefPosition.z << " m" << '\n';
        consoleOut << "WGS-84 Position: Lat = " << solution.geodeticPosition.latitude
                   << " deg, Lon = " << solution.geodeticPosition.longitude
                   << " deg, Alt = " << solution.geodeticPosition.altitude << " m" << '\n';
        consoleOut << "DOP Metrics    : HDOP = " << solution.dopHDOP << ", PDOP = " << solution.dopPDOP
                   << ", VDOP = " << solution.dopVDOP << '\n';

        std::vector<std::string> gpgsvSentences;
        if (m_nmeaGenerator.isGsvEnabled())
        {
            gpgsvSentences = NmeaGenerator::generateGpgsvSentences(m_channels, solution.ecefPosition, solution.isValid);
        }

        consoleOut << "\n--- NMEA 0183 Output Stream ---" << '\n';
        if (m_nmeaGenerator.isGgaEnabled())
        {
            consoleOut << NmeaGenerator::generateGgga(solution, static_cast<int>(activePrns.size()), receiverTime);
        }
        if (m_nmeaGenerator.isRmcEnabled())
        {
            consoleOut << NmeaGenerator::generateGprmc(solution, receiverTime);
        }
        if (m_nmeaGenerator.isGsaEnabled())
        {
            consoleOut << NmeaGenerator::generateGpgsa(solution, activePrns);
        }
        for (const std::string &gsvSentence : gpgsvSentences)
        {
            consoleOut << gsvSentence;
        }

        AsyncOutputJob consoleJob;
        consoleJob.isConsole = true;
        consoleJob.content = consoleOut.str();
        m_outputQueue.tryPush(std::move(consoleJob));

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
                const NmeaGeneratorOutput nmeaOut =
                    NmeaGenerator::generateGggaOutput(solution, static_cast<int>(activePrns.size()), receiverTime);
                m_sink->publishNmeaGeneratorOutput(nmeaOut);
            }
            if (m_nmeaGenerator.isRmcEnabled())
            {
                const NmeaGeneratorOutput nmeaOut = NmeaGenerator::generateGprmcOutput(solution, receiverTime);
                m_sink->publishNmeaGeneratorOutput(nmeaOut);
            }
            if (m_nmeaGenerator.isGsaEnabled())
            {
                const NmeaGeneratorOutput nmeaOut = NmeaGenerator::generateGpgsaOutput(solution, activePrns);
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

void Application::setSink(const std::shared_ptr<Sink> &sink)
{
    m_sink = sink;
    m_profiler.setSink(sink);
    m_navDecoder.setSink(sink);
    for (auto &channel : m_channels)
    {
        channel.setSink(sink);
    }
}

void Application::setSource(std::shared_ptr<Source> source)
{
    m_source = std::move(source);
    if (m_source && m_sink)
    {
        m_source->setSink(m_sink);
    }
}

void Application::processBlock(const ComplexFloatVector &input, uint32_t blockIndex)
{
    m_profiler.startBlock(blockIndex, static_cast<double>(blockIndex) * 0.001);

    AcquisitionResult completedAcquisition;
    if (m_acquisitionResultQueue.tryPop(completedAcquisition))
    {
        finalizeAcquisition(completedAcquisition.channelIndex);
        m_profiler.recordStageTimeMs("acquisition", completedAcquisition.correlateMs);
        m_acquisitionBusy.store(false, std::memory_order_release);
    }

    if (!m_acquisitionBusy.load(std::memory_order_acquire))
    {
        int channelToSearch = -1;
        if (!m_coldStartSweepDone)
        {
            if (m_nextAcquisitionChannel < GPS_CA_SV_COUNT)
            {
                channelToSearch = m_nextAcquisitionChannel++;
            }
            else
            {
                m_coldStartSweepDone = true;
            }
        }
        else
        {
            const int32_t reacquisitionIntervalBlocks = m_configuration.acquisitionInput.reacquisitionIntervalBlocks;
            if (reacquisitionIntervalBlocks > 0)
            {
                const int32_t slotWidth = std::max<int32_t>(1, reacquisitionIntervalBlocks / GPS_CA_SV_COUNT);
                if (blockIndex % static_cast<uint32_t>(slotWidth) == 0)
                {
                    channelToSearch = static_cast<int>((blockIndex / static_cast<uint32_t>(slotWidth)) %
                                                       static_cast<uint32_t>(GPS_CA_SV_COUNT));
                }
            }
        }

        if (channelToSearch >= 0 && m_channels[channelToSearch].isEligibleForAcquisition())
        {
            AcquisitionJob job;
            job.channelIndex = channelToSearch;
            job.input = input;
            if (m_acquisitionJobQueue.tryPush(std::move(job)))
            {
                m_acquisitionBusy.store(true, std::memory_order_release);
            }
        }
    }
    {
        const Profiler::ScopedTimer trackTimer(m_profiler, "tracking");
        trackSatellites(input);
    }
    {
        double earlyLatePromptGenTotalMs = 0.0;
        double numericOscillatorTotalMs = 0.0;
        double accumulatorTotalMs = 0.0;
        for (const auto &channel : m_channels)
        {
            float earlyLatePromptGenMs = 0.0f;
            float numericOscillatorMs = 0.0f;
            float accumulatorMs = 0.0f;
            channel.getTrackingSubStageTimings(&earlyLatePromptGenMs, &numericOscillatorMs, &accumulatorMs);
            earlyLatePromptGenTotalMs += earlyLatePromptGenMs;
            numericOscillatorTotalMs += numericOscillatorMs;
            accumulatorTotalMs += accumulatorMs;
        }
        double maxWorkerMs = 0.0;
        for (const double workerMs : m_workerDurationMs)
        {
            maxWorkerMs = std::max(maxWorkerMs, workerMs);
        }
        m_profiler.recordTrackingSubStageTimings(
            earlyLatePromptGenTotalMs, numericOscillatorTotalMs, accumulatorTotalMs, maxWorkerMs);
    }
    {
        const Profiler::ScopedTimer navTimer(m_profiler, "navDecode");
        updateChannelNavigation();
    }
    const int32_t fixOutputIntervalBlocks = m_configuration.pvtSolverInput.fixOutputIntervalBlocks;
    if (fixOutputIntervalBlocks <= 0 || blockIndex % static_cast<uint32_t>(fixOutputIntervalBlocks) == 0)
    {
        const Profiler::ScopedTimer pvtTimer(m_profiler, "pvtSolve");
        ReceiverPvtSolution solution{};
        computeNavigationSolution(solution);
    }
    m_profiler.finishBlock();
}

void Application::exportTelemetryJson(const std::string &filepath,
                                      const ReceiverPvtSolution &solution,
                                      double utcTimeSec)
{
    std::ostringstream file;

    std::vector<int> activePrns;
    for (auto &channel : m_channels)
    {
        if (channel.isAcquired())
        {
            activePrns.push_back(channel.m_svId);
        }
    }

    file << "{\n";
    file << "  \"timestamp\": " << utcTimeSec << ",\n";
    file << R"(  "software": ")" << SOFTWARE_NAME << " " << SOFTWARE_VERSION << "\",\n";
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
        float peakVal = 0.0f;
        float peakFreq = 0.0f;
        float meanVal = 0.0f;
        float cn0 = 0.0f;
        float peakRatio = 0.0f;
        m_channels[i].getAcquisitionResults(&peakIndex, &peakVal, &peakFreq, &meanVal, &cn0, &peakRatio);

        bool hasPosition = false;
        double az = 0.0;
        double el = 0.0;

        if (solution.isValid && m_channels[i].hasCompleteEphemeris())
        {
            const size_t promptCount = m_channels[i].getPromptHistory().size();
            const size_t subframeStartSample = m_channels[i].getLastSubframeStartSample();
            if (promptCount >= subframeStartSample)
            {
                const double subframeStartTow = m_channels[i].getLastSubframeTow() - 6.0;
                const double elapsedSeconds =
                    static_cast<double>(promptCount - subframeStartSample) * GPS_CA_CODE_PERIOD_SEC;
                const double transmitTime = subframeStartTow + elapsedSeconds;

                const SatelliteOrbit orbit =
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
    file << "    \""
         << stripTrailingNewlines(
                NmeaGenerator::generateGgga(solution, static_cast<int>(activePrns.size()), utcTimeSec))
         << "\",\n";
    file << "    \"" << stripTrailingNewlines(NmeaGenerator::generateGprmc(solution, utcTimeSec)) << "\",\n";
    file << "    \"" << stripTrailingNewlines(NmeaGenerator::generateGpgsa(solution, activePrns)) << "\"\n";
    file << "  ]\n";
    file << "}\n";

    AsyncOutputJob job;
    job.isConsole = false;
    job.filePath = filepath;
    job.content = file.str();
    m_outputQueue.tryPush(std::move(job));
}

void Application::initializeChannels()
{
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        m_channels[i].m_svId = i + 1;
    }
}
