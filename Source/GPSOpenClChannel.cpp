#include "GPSOpenClChannel.h"
#include "GPSOpenClCommon.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <utility>

using namespace GPSOpenCl;

Channel::Channel()
    : m_svId(0),
      m_acquisitionPeakIndex(0),
      m_acquisitionPeakValue(0.0f),
      m_acquisitionPeakFrequency(0.0f),
      m_acquisitionMeanValue(0.0f),
      m_acquisitionCN0(0.0f),
      m_acquisitionPeakRatio(0.0f),
      m_acquisitionProcessingGain(static_cast<float>(10.0 * std::log10(GPS_CA_CODE_FREQUENCY_HZ / GPS_CA_CODE_LENGTH))),
      m_isAcquired(false),
      m_tracking(nullptr),
      m_lastRawCodePhaseForDrift(0.0f),
      m_cumulativeDriftChips(0.0f),
      m_bitSyncPhase(-1),

      m_navBitOffset(0),
      m_seenSubframeMask(0),
      m_accumulatedEphemeris(),
      m_lastSubframeTow(0.0),
      m_lastSubframeStartSample(0),
      m_state(ChannelState::Acquiring),
      m_confirmProgress(0),
      m_lossProgress(0),
      m_blocksInConfirming(0),
      m_carrierLockThreshold(0.3f),
      m_codeLockRatioTolerance(0.3f),
      m_confirmDebounceBlocks(50),
      m_confirmTimeoutBlocks(200),
      m_lossDebounceBlocks(200)
{
}

Channel::~Channel() = default;

void Channel::insertAcquisitionMetrics(float peakValue, int peakIndex, float peakFrequency, float meanValue)
{
    const std::lock_guard<std::mutex> lock(m_acquisitionMetricsMutex);
    if (peakValue > m_acquisitionPeakValue)
    {
        m_acquisitionPeakValue = peakValue;
        m_acquisitionPeakIndex = peakIndex;
        m_acquisitionPeakFrequency = peakFrequency;
    }

    m_acquisitionMeanValue += meanValue;
    if (m_acquisitionMeanValue != 0.0f)
    {
        m_acquisitionPeakRatio = m_acquisitionPeakValue / m_acquisitionMeanValue;
        m_acquisitionCN0 = 10.0f * std::log10(m_acquisitionPeakRatio) + m_acquisitionProcessingGain;
    }
    else
    {
        m_acquisitionPeakRatio = 0.0f;
        m_acquisitionCN0 = 0.0f;
    }
}

void Channel::resetAcquisitionMetrics()
{
    const std::lock_guard<std::mutex> lock(m_acquisitionMetricsMutex);
    m_acquisitionPeakIndex = 0;
    m_acquisitionPeakValue = 0.0f;
    m_acquisitionPeakFrequency = 0.0f;
    m_acquisitionMeanValue = 0.0f;
    m_acquisitionCN0 = 0.0f;
    m_acquisitionPeakRatio = 0.0f;
}

void Channel::getAcquisitionResults(int *peakIndex,
                                    float *peakValue,
                                    float *peakFrequency,
                                    float *meanValue,
                                    float *cno,
                                    float *peakRatio) const
{
    const std::lock_guard<std::mutex> lock(m_acquisitionMetricsMutex);
    *peakIndex = m_acquisitionPeakIndex;
    *peakValue = m_acquisitionPeakValue;
    *peakFrequency = m_acquisitionPeakFrequency;
    *meanValue = m_acquisitionMeanValue;
    *cno = m_acquisitionCN0;
    *peakRatio = m_acquisitionPeakRatio;
}

bool Channel::isAcquired() const
{
    return m_isAcquired;
}

void Channel::setAcquired(bool acquired)
{
    m_isAcquired = acquired;
}

void Channel::setSink(std::shared_ptr<Sink> sink)
{
    m_sink = std::move(sink);
}

bool Channel::getTrackingOutput(TrackingOutput *out) const
{
    if (m_tracking == nullptr)
    {
        return false;
    }
    *out = m_tracking->getTrackingOutput(m_svId);
    return true;
}

void Channel::initTracking(const Settings::Configuration &conf, float dopplerHz, float codePhaseChips)
{
    if (m_tracking == nullptr)
    {
        m_tracking = std::make_unique<Tracking>(conf);
    }
    m_tracking->setTimingEnabled(m_trackingTimingEnabled);
    m_tracking->initTrackingState(dopplerHz, codePhaseChips);
    resetNavigationState();

    m_carrierLockThreshold = static_cast<float>(conf.trackingInput.carrierLockThreshold);
    m_codeLockRatioTolerance = static_cast<float>(conf.trackingInput.codeLockRatioTolerance);
    m_confirmDebounceBlocks = conf.trackingInput.confirmDebounceBlocks;
    m_confirmTimeoutBlocks = conf.trackingInput.confirmTimeoutBlocks;
    m_lossDebounceBlocks = conf.trackingInput.lossDebounceBlocks;

    m_state = ChannelState::Confirming;
    m_confirmProgress = 0;
    m_lossProgress = 0;
    m_blocksInConfirming = 0;
}

void Channel::resetNavigationState()
{
    m_promptHistory.clear();
    m_promptHistory.reserve(60'000);
    m_codePhaseHistory.clear();
    m_codePhaseHistory.reserve(60'000);
    m_cumulativeDriftChipsHistory.clear();
    m_cumulativeDriftChipsHistory.reserve(60'000);
    m_lastRawCodePhaseForDrift = 0.0f;
    m_cumulativeDriftChips = 0.0f;
    m_bitSyncPhase = -1;
    m_bitSyncSearchPositions.clear();
    m_navBitOffset = 0;
    m_seenSubframeMask = 0;
    m_accumulatedEphemeris = GpsEphemeris();
    m_lastSubframeTow = 0.0;
    m_lastSubframeStartSample = 0;
}

void Channel::trackBlock(const ComplexFloatVector &input)
{
    if ((m_tracking != nullptr) && m_isAcquired)
    {
        ComplexFloatVector *promptOutput = (m_state == ChannelState::Tracking) ? &m_promptHistory : nullptr;
        m_tracking->doWork(input, m_svId, promptOutput, static_cast<uint32_t>(m_state));
        if (m_state == ChannelState::Tracking)
        {
            const float rawCodePhase = m_tracking->getCodePhaseChips();
            m_codePhaseHistory.push_back(rawCodePhase);

            if (!m_cumulativeDriftChipsHistory.empty())
            {
                const float delta = std::fmod(rawCodePhase - m_lastRawCodePhaseForDrift + 1534.5f, 1023.0f) - 511.5f;
                m_cumulativeDriftChips += delta;
            }
            m_lastRawCodePhaseForDrift = rawCodePhase;
            m_cumulativeDriftChipsHistory.push_back(m_cumulativeDriftChips);
        }
        evaluateLockState();
    }
}

void Channel::getTrackingSubStageTimings(float *earlyLatePromptGenMs,
                                         float *numericOscillatorMs,
                                         float *accumulatorMs) const
{
    if (m_tracking != nullptr)
    {
        m_tracking->getSubStageTimings(earlyLatePromptGenMs, numericOscillatorMs, accumulatorMs);
    }
    else
    {
        *earlyLatePromptGenMs = 0.0f;
        *numericOscillatorMs = 0.0f;
        *accumulatorMs = 0.0f;
    }
}

ChannelState Channel::computeNextState(ChannelState current,
                                       bool goodBlock,
                                       int &confirmProgress,
                                       int &lossProgress,
                                       int &blocksInConfirming,
                                       int confirmDebounceBlocks,
                                       int confirmTimeoutBlocks,
                                       int lossDebounceBlocks)
{
    if (current == ChannelState::Confirming)
    {
        confirmProgress = goodBlock ? confirmProgress + 1 : std::max(0, confirmProgress - 1);
        blocksInConfirming++;

        if (confirmProgress >= confirmDebounceBlocks)
        {
            lossProgress = 0;
            return ChannelState::Tracking;
        }
        if (blocksInConfirming >= confirmTimeoutBlocks)
        {
            return ChannelState::Acquiring;
        }
        return ChannelState::Confirming;
    }

    if (current == ChannelState::Tracking)
    {
        lossProgress = goodBlock ? std::max(0, lossProgress - 1) : lossProgress + 1;

        if (lossProgress >= lossDebounceBlocks)
        {
            return ChannelState::Acquiring;
        }
        return ChannelState::Tracking;
    }

    return current;
}

void Channel::evaluateLockState()
{
    const bool good = m_tracking->getCarrierLockIndicator() >= m_carrierLockThreshold &&
        std::fabs(m_tracking->getCodeLockRatio() - 1.0f) <= m_codeLockRatioTolerance;

    const ChannelState previous = m_state;
    m_state = computeNextState(m_state,
                               good,
                               m_confirmProgress,
                               m_lossProgress,
                               m_blocksInConfirming,
                               m_confirmDebounceBlocks,
                               m_confirmTimeoutBlocks,
                               m_lossDebounceBlocks);

    if (previous == ChannelState::Confirming && m_state == ChannelState::Acquiring)
    {
        m_isAcquired = false;
        resetNavigationState();
        std::ostringstream msg;
        msg << "SV ID " << m_svId << " confirmation TIMED OUT (lock: carrier=" << m_tracking->getCarrierLockIndicator()
            << " code=" << m_tracking->getCodeLockRatio() << "), back to acquiring\n";
        std::cout << msg.str();
    }
    else if (previous == ChannelState::Tracking && m_state == ChannelState::Acquiring)
    {
        m_isAcquired = false;
        resetNavigationState();
        std::cout << "SV ID " + std::to_string(m_svId) + " LOST LOCK, back to acquiring\n";
    }
    else if (previous == ChannelState::Confirming && m_state == ChannelState::Tracking)
    {
        std::cout << "SV ID " + std::to_string(m_svId) + " tracking CONFIRMED\n";
    }
}

void Channel::compactNavigationHistory()
{
    if (m_promptHistory.size() < NAV_HISTORY_COMPACT_THRESHOLD)
    {
        return;
    }

    size_t dropSamples = 0;
    if (m_bitSyncPhase >= 0)
    {
        const auto phase = static_cast<size_t>(m_bitSyncPhase);
        const size_t currentReadSample = phase + (m_navBitOffset * 20);
        const size_t anchorSample = std::min(m_lastSubframeStartSample, currentReadSample);
        const size_t margin = phase + 40;
        if (anchorSample > margin)
        {
            dropSamples = ((anchorSample - margin) / 20) * 20;
        }
    }
    else if (!m_bitSyncSearchPositions.empty())
    {
        size_t minSearchedBits = m_bitSyncSearchPositions[0];
        for (const size_t position : m_bitSyncSearchPositions)
        {
            minSearchedBits = std::min(minSearchedBits, position);
        }
        if (minSearchedBits > 2)
        {
            dropSamples = (minSearchedBits - 2) * 20;
        }
    }

    if (dropSamples < NAV_HISTORY_COMPACT_MIN_DROP || dropSamples > m_promptHistory.size())
    {
        return;
    }

    const auto dropOffset = static_cast<std::ptrdiff_t>(dropSamples);
    m_promptHistory.erase(m_promptHistory.begin(), m_promptHistory.begin() + dropOffset);
    m_codePhaseHistory.erase(m_codePhaseHistory.begin(), m_codePhaseHistory.begin() + dropOffset);
    m_cumulativeDriftChipsHistory.erase(m_cumulativeDriftChipsHistory.begin(),
                                        m_cumulativeDriftChipsHistory.begin() + dropOffset);

    const size_t dropBits = dropSamples / 20;
    if (m_bitSyncPhase >= 0)
    {
        m_lastSubframeStartSample -= dropSamples;
        m_navBitOffset -= dropBits;
    }
    else
    {
        for (size_t &position : m_bitSyncSearchPositions)
        {
            position = (position > dropBits) ? position - dropBits : 0;
        }
    }
}

bool Channel::updateNavigation(NavigationDecoder &decoder)
{
    compactNavigationHistory();

    GpsEphemeris ephem = GpsEphemeris();
    size_t subframeStartSample = 0;
    if (!decoder.processPromptSignal(m_svId,
                                     m_promptHistory,
                                     m_bitSyncPhase,
                                     m_bitSyncSearchPositions,
                                     m_navBitOffset,
                                     ephem,
                                     subframeStartSample,
                                     &m_codePhaseHistory))
    {
        return hasCompleteEphemeris();
    }

    m_accumulatedEphemeris.svId = ephem.svId;
    m_accumulatedEphemeris.tow = ephem.tow;
    m_accumulatedEphemeris.subframeId = ephem.subframeId;

    switch (ephem.subframeId)
    {
        case 1:
            m_accumulatedEphemeris.weekNumber = ephem.weekNumber;
            m_accumulatedEphemeris.toc = ephem.toc;
            m_accumulatedEphemeris.af0 = ephem.af0;
            m_accumulatedEphemeris.af1 = ephem.af1;
            m_accumulatedEphemeris.af2 = ephem.af2;
            m_accumulatedEphemeris.tgd = ephem.tgd;
            m_seenSubframeMask |= 0x1;
            break;
        case 2:
            m_accumulatedEphemeris.toe = ephem.toe;
            m_accumulatedEphemeris.sqrtA = ephem.sqrtA;
            m_accumulatedEphemeris.e = ephem.e;
            m_accumulatedEphemeris.M0 = ephem.M0;
            m_accumulatedEphemeris.deltaN = ephem.deltaN;
            m_accumulatedEphemeris.Cuc = ephem.Cuc;
            m_accumulatedEphemeris.Cus = ephem.Cus;
            m_accumulatedEphemeris.Crs = ephem.Crs;
            m_seenSubframeMask |= 0x2;
            break;
        case 3:
            m_accumulatedEphemeris.i0 = ephem.i0;
            m_accumulatedEphemeris.idot = ephem.idot;
            m_accumulatedEphemeris.omega0 = ephem.omega0;
            m_accumulatedEphemeris.omegaDot = ephem.omegaDot;
            m_accumulatedEphemeris.omega = ephem.omega;
            m_accumulatedEphemeris.Cic = ephem.Cic;
            m_accumulatedEphemeris.Cis = ephem.Cis;
            m_accumulatedEphemeris.Crc = ephem.Crc;
            m_seenSubframeMask |= 0x4;
            break;
        default:
            break;
    }

    m_lastSubframeTow = ephem.tow;
    m_lastSubframeStartSample = subframeStartSample;
    m_accumulatedEphemeris.isValid = hasCompleteEphemeris();

    return hasCompleteEphemeris();
}

bool Channel::hasCompleteEphemeris() const
{
    return (m_seenSubframeMask & 0x7) == 0x7;
}
