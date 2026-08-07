#include "Application/GPSOpenClApplication.hpp"

#include "Pvt/GPSOpenClAtmosphericCorrections.hpp"
#include "Pvt/GPSOpenClMeasurementAssembler.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <thread>
#include <utility>

using namespace GPSOpenCl;

Application::Application(const Settings::Configuration &conf)
    : m_acquisition(nullptr),

      m_configuration(conf),
      m_code(nullptr),
      m_gpu(std::make_unique<SpectrumEngine>()),
      m_pvtSolver(conf.pvtSolverInput),
      m_navDecoder(conf.navDecoderInput),
      m_nmeaGenerator(conf.nmeaGeneratorInput),
      m_trackingPool([this](int slot) { trackOneActiveChannel(slot); }, GPS_CA_SV_COUNT)
{
    std::cout << SOFTWARE_NAME << " " << SOFTWARE_VERSION << " started to run" << '\n';

    m_code = std::make_unique<CaCodeGenerator>(m_configuration);
    m_code->createLookupTable(m_gpu.get());

    m_acquisition = std::make_unique<Acquisition>(m_configuration);

    m_pvtSolver.setIonosphericParams(conf.atmosphericInput);
    m_profiler.setEnabled(conf.profilerInput.enabled != 0);

    initializeChannels();
    m_acquisitionThread = std::thread([this] { acquisitionWorkerLoop(); });
}

Application::~Application()
{
    m_acquisitionJobQueue.finish();
    if (m_acquisitionThread.joinable())
    {
        m_acquisitionThread.join();
    }
}

void Application::searchOneChannel(const ComplexFloatVector &input, int channelIndex)
{
    if (!m_channels[channelIndex].isEligibleForAcquisition())
    {
        return;
    }

    m_acquisition->correlate(input, m_gpu.get(), m_code.get(), &m_channels[channelIndex]);
    finalizeAcquisition(channelIndex, 0.0);
}

void Application::finalizeAcquisition(int channelIndex, double correlateMs)
{
    Channel &channel = m_channels[channelIndex];

    int peakIndex = 0;
    float peakValue = 0.0f;
    float peakFreq = 0.0f;
    float meanValue = 0.0f;
    float cn0 = 0.0f;
    float peakRatio = 0.0f;
    channel.getAcquisitionResults(&peakIndex, &peakValue, &peakFreq, &meanValue, &cn0, &peakRatio);

    const float dopplerHz = -peakFreq;

    std::ostringstream acqMsg;
    acqMsg << "SV ID " << channel.svId << " C/N0 : " << cn0 << "\n";

    const auto acquisitionCn0ThresholdDbHz =
        static_cast<float>(m_configuration.acquisitionInput.acquisitionCn0ThresholdDbHz);
    if (cn0 >= acquisitionCn0ThresholdDbHz)
    {
        channel.setAcquired(true);
        const int numberOfSamplesPerCode = m_configuration.acquisitionInput.numberOfSamplesPerCode;
        auto numSamplesFloat = static_cast<float>(numberOfSamplesPerCode);
        const int reflectedPeakIndex =
            (numberOfSamplesPerCode > 0) ? ((numberOfSamplesPerCode - peakIndex) % numberOfSamplesPerCode) : 0;
        const float codePhaseChips = (numSamplesFloat > 0.0f)
            ? (static_cast<float>(reflectedPeakIndex) / numSamplesFloat) * GPS_CA_CODE_LENGTH
            : 0.0f;
        channel.initTracking(m_configuration, dopplerHz, codePhaseChips);
        acqMsg << "--> SV ID " << channel.svId << " ACQUIRED! (C/N0: " << cn0 << " dB-Hz, Doppler: " << dopplerHz
               << " Hz)" << '\n';
    }

    m_telemetryExporter.pushConsole(acqMsg.str());

    if (m_sink)
    {
        AcquisitionOutput acqOut;
        acqOut.structVersion = STRUCT_VERSION_2;
        acqOut.correlateMs = correlateMs;
        acqOut.prn = channel.svId;
        acqOut.peakIndex = peakIndex;
        acqOut.peakValue = static_cast<double>(peakValue);
        acqOut.peakFrequencyHz = static_cast<double>(dopplerHz);
        acqOut.meanValue = static_cast<double>(meanValue);
        acqOut.cnoDbHz = static_cast<double>(cn0);
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
        try
        {
            auto start = std::chrono::high_resolution_clock::now();
            m_acquisition->correlate(job.input, m_gpu.get(), m_code.get(), &m_channels[job.channelIndex]);
            auto end = std::chrono::high_resolution_clock::now();

            AcquisitionResult result;
            result.channelIndex = job.channelIndex;
            result.correlateMs = std::chrono::duration<double, std::milli>(end - start).count();
            result.recycledInput = std::move(job.input);
            m_acquisitionResultQueue.tryPush(std::move(result));
        }
        catch (const std::exception &e)
        {
            std::cerr << "Acquisition worker: channel " << (job.channelIndex + 1) << " search threw: " << e.what()
                      << '\n';
            AcquisitionResult result;
            result.channelIndex = job.channelIndex;
            m_acquisitionResultQueue.tryPush(std::move(result));
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

void Application::trackSatellites(const ComplexFloatVector &input)
{
    m_activeChannels.clear();
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        if (m_channels[i].isTrackingLoopActive())
        {
            m_activeChannels.push_back(i);
        }
    }

    m_currentTrackInput = &input;
    m_trackingPool.run(static_cast<int>(m_activeChannels.size()));
    m_currentTrackInput = nullptr;

    publishTrackingOutputs();
}

void Application::trackOneActiveChannel(int slot)
{
    const int channelIndex = m_activeChannels[static_cast<size_t>(slot)];
    try
    {
        m_channels[channelIndex].trackBlock(*m_currentTrackInput);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Channel " << m_channels[channelIndex].svId << " tracking threw: " << e.what() << '\n';
    }
}

void Application::publishTrackingOutputs()
{
    for (const int channelIndex : m_activeChannels)
    {
        std::string message = m_channels[channelIndex].takePendingStateMessage();
        if (!message.empty())
        {
            m_telemetryExporter.pushConsole(std::move(message));
        }
    }

    if (!m_sink)
    {
        return;
    }

    const auto interval = static_cast<uint32_t>(std::max(m_configuration.trackingInput.telemetryIntervalBlocks, 1));
    if (m_currentBlockIndex % interval != 0)
    {
        return;
    }

    for (const int channelIndex : m_activeChannels)
    {
        TrackingOutput trackOut{};
        if (m_channels[channelIndex].getTrackingOutput(&trackOut))
        {
            m_sink->publishTrackingOutput(trackOut);
        }
    }
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
        diag.svId = channel.svId;
        diag.bitSyncPhase = channel.getBitSyncPhase();
        diag.subframeStartTow = channel.getLastSubframeTow() - GPS_NAV_SUBFRAME_DURATION_SEC;
        diag.subframeStartSample = channel.getLastSubframeStartSample();
        diag.codePhaseAtSubframeStart = channel.getCodePhaseAtSample(diag.subframeStartSample);
        if (!MeasurementAssembler::computeElapsedSecondsSincePromptStart(
                promptCount, diag.subframeStartSample, diag.elapsedSeconds))
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
    MeasurementAssembler::Measurements measurements;
    if (!MeasurementAssembler::assemble(
            m_channels, GPS_CA_SV_COUNT, m_pvtSolver.getReferenceEcef(), isReferencePositionTrusted(), measurements))
    {
        static int insufficientSatCount = 0;
        if (insufficientSatCount++ % 20 == 0)
        {
            m_telemetryExporter.pushConsole(
                "Navigation Solution Error: Less than 4 satellites with a complete "
                "decoded ephemeris (" +
                std::to_string(measurements.prns.size()) + " ready).\n");
        }
        solution.isValid = false;
        m_lastPseudorangeSamples.clear();
        return false;
    }

    const std::vector<GpsEphemeris> &ephemerides = measurements.ephemerides;
    const std::vector<double> &transmitTimes = measurements.transmitTimesSec;
    const std::vector<double> &measuredPseudoranges = measurements.pseudorangesMeters;
    const std::vector<int> &activePrns = measurements.prns;
    const double receiverTime = measurements.receiverTimeSec;

    m_lastPseudorangeSamples.clear();
    for (size_t i = 0; i < activePrns.size(); i++)
    {
        m_lastPseudorangeSamples.push_back({activePrns[i], transmitTimes[i], measuredPseudoranges[i]});
    }

    if (m_navDecoder.hasIonosphericParams())
    {
        m_pvtSolver.setIonosphericParams(m_navDecoder.getIonosphericParams());
    }

    const bool success = m_pvtSolver.solvePosition(ephemerides, measuredPseudoranges, transmitTimes, solution);
    if (!success && !m_pvtSolver.hasValidFix())
    {
        m_seedSolveFailures++;
    }
    if (success)
    {
        std::ostringstream consoleOut;
        consoleOut << "\n=============================================" << '\n';
        consoleOut << "   GPS PVT Position Solution Computed        " << '\n';
        consoleOut << "=============================================" << '\n';
        consoleOut << "ECEF Position : X = " << solution.ecefPosition.x << " m, Y = " << solution.ecefPosition.y
                   << " m, Z = " << solution.ecefPosition.z << " m" << '\n';
        consoleOut << "WGS-84 Position: Lat = " << solution.geodeticPosition.latitudeDeg
                   << " deg, Lon = " << solution.geodeticPosition.longitudeDeg
                   << " deg, Alt = " << solution.geodeticPosition.altitudeMeters << " m" << '\n';
        consoleOut << "DOP Metrics    : HDOP = " << solution.dopHDOP << ", PDOP = " << solution.dopPDOP
                   << ", VDOP = " << solution.dopVDOP << '\n';

        std::vector<std::string> gpgsvSentences;
        if (m_nmeaGenerator.isGsvEnabled())
        {
            gpgsvSentences = NmeaGenerator::generateGpgsvSentences(m_channels, solution.ecefPosition, solution.isValid);
        }

        const int gpsWeekNumber = ephemerides.empty() ? 0 : ephemerides[0].weekNumber;
        const std::string ggaSentence =
            NmeaGenerator::generateGgga(solution, static_cast<int>(activePrns.size()), receiverTime, gpsWeekNumber);
        const std::string rmcSentence = NmeaGenerator::generateGprmc(solution, receiverTime, gpsWeekNumber);
        const std::string gsaSentence = NmeaGenerator::generateGpgsa(solution, activePrns);

        consoleOut << "\n--- NMEA 0183 Output Stream ---" << '\n';
        if (m_nmeaGenerator.isGgaEnabled())
        {
            consoleOut << ggaSentence;
        }
        if (m_nmeaGenerator.isRmcEnabled())
        {
            consoleOut << rmcSentence;
        }
        if (m_nmeaGenerator.isGsaEnabled())
        {
            consoleOut << gsaSentence;
        }
        for (const std::string &gsvSentence : gpgsvSentences)
        {
            consoleOut << gsvSentence;
        }

        m_telemetryExporter.pushConsole(consoleOut.str());

        exportTelemetryJson("telemetry_stream.json", solution, receiverTime, ggaSentence, rmcSentence, gsaSentence);

        if (m_sink)
        {
            m_sink->publishPvtSolverOutput(PVTSolver::solutionToOutput(solution));

            const AtmosphericInput ionoParams =
                m_navDecoder.hasIonosphericParams() ? m_navDecoder.getIonosphericParams() : AtmosphericInput{};
            for (size_t i = 0; i < ephemerides.size(); i++)
            {
                const SatelliteOrbit orbit = PVTSolver::computeSatelliteOrbit(ephemerides[i], transmitTimes[i]);
                const AtmosphericOutput atmoOut = AtmosphericCorrections::computeCorrections(activePrns[i],
                                                                                             solution.geodeticPosition,
                                                                                             solution.ecefPosition,
                                                                                             orbit.position,
                                                                                             receiverTime,
                                                                                             ionoParams);
                m_sink->publishAtmosphericOutput(atmoOut);
            }

            if (m_nmeaGenerator.isGgaEnabled())
            {
                m_sink->publishNmeaGeneratorOutput(NmeaGenerator::outputFromSentence(ggaSentence));
            }
            if (m_nmeaGenerator.isRmcEnabled())
            {
                m_sink->publishNmeaGeneratorOutput(NmeaGenerator::outputFromSentence(rmcSentence));
            }
            if (m_nmeaGenerator.isGsaEnabled())
            {
                m_sink->publishNmeaGeneratorOutput(NmeaGenerator::outputFromSentence(gsaSentence));
            }
            for (const std::string &gsvSentence : gpgsvSentences)
            {
                m_sink->publishNmeaGeneratorOutput(NmeaGenerator::outputFromSentence(gsvSentence));
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

void Application::processBlock(const ComplexFloatVector &input, uint32_t blockIndex)
{
    m_currentBlockIndex = blockIndex;
    m_profiler.startBlock(blockIndex, static_cast<double>(blockIndex) * GPS_CA_CODE_PERIOD_SEC);

    AcquisitionResult completedAcquisition;
    if (m_acquisitionResultQueue.tryPop(completedAcquisition))
    {
        finalizeAcquisition(completedAcquisition.channelIndex, completedAcquisition.correlateMs);
        m_profiler.recordStageTimeMs(Profiler::Stage::Acquisition, completedAcquisition.correlateMs);
        m_acqInputPool = std::move(completedAcquisition.recycledInput);
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
            job.input = std::move(m_acqInputPool);
            job.input.assign(input.begin(), input.end());
            if (m_acquisitionJobQueue.tryPush(std::move(job)))
            {
                m_acquisitionBusy.store(true, std::memory_order_release);
            }
        }
    }
    {
        const Profiler::ScopedTimer trackTimer(m_profiler, Profiler::Stage::Tracking);
        trackSatellites(input);
    }
    if (m_profiler.isEnabled())
    {
        double correlatorTotalMs = 0.0;
        for (const int channelIndex : m_activeChannels)
        {
            correlatorTotalMs += m_channels[channelIndex].getTrackingCorrelatorTimeMs();
        }
        const double maxWorkerMs = m_trackingPool.maxWorkerDurationMs();
        m_profiler.recordTrackingSubStageTimings(correlatorTotalMs, maxWorkerMs);
    }
    {
        const Profiler::ScopedTimer navTimer(m_profiler, Profiler::Stage::NavDecode);
        updateChannelNavigation();
    }
    const int32_t fixOutputIntervalBlocks = m_configuration.pvtSolverInput.fixOutputIntervalBlocks;
    if (fixOutputIntervalBlocks <= 0 || blockIndex % static_cast<uint32_t>(fixOutputIntervalBlocks) == 0)
    {
        const Profiler::ScopedTimer pvtTimer(m_profiler, Profiler::Stage::PvtSolve);
        ReceiverPvtSolution solution{};
        computeNavigationSolution(solution);
    }
    m_profiler.finishBlock();
}

void Application::exportTelemetryJson(const std::string &filepath,
                                      const ReceiverPvtSolution &solution,
                                      double utcTimeSec,
                                      const std::string &ggaSentence,
                                      const std::string &rmcSentence,
                                      const std::string &gsaSentence)
{
    auto snapshot = std::make_unique<TelemetryExporter::TelemetrySnapshot>();
    snapshot->solution = solution;
    snapshot->utcTimeSec = utcTimeSec;
    snapshot->ggaSentence = ggaSentence;
    snapshot->rmcSentence = rmcSentence;
    snapshot->gsaSentence = gsaSentence;

    snapshot->satellites.reserve(GPS_CA_SV_COUNT);
    for (auto &channel : m_channels)
    {
        TelemetryExporter::SatelliteTelemetry sat;
        sat.prn = channel.svId;
        sat.acquired = channel.isAcquired();
        if (sat.acquired)
        {
            snapshot->activePrns.push_back(sat.prn);
        }

        int peakIndex = 0;
        float peakVal = 0.0f;
        float peakFreq = 0.0f;
        float meanVal = 0.0f;
        float cn0 = 0.0f;
        float peakRatio = 0.0f;
        channel.getAcquisitionResults(&peakIndex, &peakVal, &peakFreq, &meanVal, &cn0, &peakRatio);
        sat.cn0 = cn0;
        sat.doppler = -peakFreq;

        if (solution.isValid && channel.hasCompleteEphemeris())
        {
            const size_t promptCount = channel.getPromptHistory().size();
            const size_t subframeStartSample = channel.getLastSubframeStartSample();
            if (promptCount >= subframeStartSample)
            {
                const double subframeStartTow = channel.getLastSubframeTow() - GPS_NAV_SUBFRAME_DURATION_SEC;
                const double elapsedSeconds =
                    static_cast<double>(promptCount - subframeStartSample) * GPS_CA_CODE_PERIOD_SEC;
                sat.transmitTime = subframeStartTow + elapsedSeconds;
                sat.ephemeris = channel.getAccumulatedEphemeris();
                sat.computeOrbit = true;
            }
        }

        snapshot->satellites.push_back(sat);
    }

    m_telemetryExporter.pushJsonFile(filepath, std::move(snapshot));
}

void Application::initializeChannels()
{
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        m_channels[i].svId = i + 1;
        m_channels[i].setTrackingTimingEnabled(m_profiler.isEnabled());
    }
}
