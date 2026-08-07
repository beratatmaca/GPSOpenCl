#include "Tracking/GPSOpenClChannel.hpp"
#include "Common/GPSOpenClCommon.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <utility>

using namespace GPSOpenCl;

Channel::Channel()
    : m_acquisitionProcessingGain(static_cast<float>(10.0 * std::log10(GPS_CA_CODE_FREQUENCY_HZ / GPS_CA_CODE_LENGTH))),
      m_tracking(nullptr),
      m_accumulatedEphemeris{}
{
}

Channel::~Channel() = default;

void Channel::insertAcquisitionMetrics(float peakValue, int peakIndex, float peakFrequencyHz, float meanValue)
{
    const std::lock_guard<std::mutex> lock(m_acquisitionMetricsMutex);
    if (peakValue > m_acquisitionPeakValue)
    {
        m_acquisitionPeakValue = peakValue;
        m_acquisitionPeakIndex = peakIndex;
        m_acquisitionPeakFrequencyHz = peakFrequencyHz;
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
    m_acquisitionPeakFrequencyHz = 0.0f;
    m_acquisitionMeanValue = 0.0f;
    m_acquisitionCN0 = 0.0f;
    m_acquisitionPeakRatio = 0.0f;
}

void Channel::getAcquisitionResults(int *peakIndex,
                                    float *peakValue,
                                    float *peakFrequencyHz,
                                    float *meanValue,
                                    float *cnoDbHz,
                                    float *peakRatio) const
{
    const std::lock_guard<std::mutex> lock(m_acquisitionMetricsMutex);
    *peakIndex = m_acquisitionPeakIndex;
    *peakValue = m_acquisitionPeakValue;
    *peakFrequencyHz = m_acquisitionPeakFrequencyHz;
    *meanValue = m_acquisitionMeanValue;
    *cnoDbHz = m_acquisitionCN0;
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
    *out = m_tracking->getTrackingOutput(svId);
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
        m_tracking->doWork(input, svId, promptOutput, static_cast<uint32_t>(m_state));
        if (m_state == ChannelState::Tracking)
        {
            const float rawCodePhase = m_tracking->getCodePhaseChips();
            m_codePhaseHistory.push_back(rawCodePhase);

            if (!m_cumulativeDriftChipsHistory.empty())
            {
                const float halfCode = static_cast<float>(GPS_CA_CODE_LENGTH) / 2.0f;
                const float delta = std::fmod(rawCodePhase - m_lastRawCodePhaseForDrift +
                                                  static_cast<float>(GPS_CA_CODE_LENGTH) + halfCode,
                                              static_cast<float>(GPS_CA_CODE_LENGTH)) -
                    halfCode;
                m_cumulativeDriftChips += delta;
            }
            m_lastRawCodePhaseForDrift = rawCodePhase;
            m_cumulativeDriftChipsHistory.push_back(m_cumulativeDriftChips);
        }
        evaluateLockState();
    }
}

float Channel::getTrackingCorrelatorTimeMs() const
{
    return (m_tracking != nullptr) ? m_tracking->getCorrelatorTimeMs() : 0.0f;
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
        msg << "SV ID " << svId << " confirmation TIMED OUT (lock: carrier=" << m_tracking->getCarrierLockIndicator()
            << " code=" << m_tracking->getCodeLockRatio() << "), back to acquiring\n";
        m_pendingStateMessage += msg.str();
    }
    else if (previous == ChannelState::Tracking && m_state == ChannelState::Acquiring)
    {
        m_isAcquired = false;
        resetNavigationState();
        m_pendingStateMessage += "SV ID " + std::to_string(svId) + " LOST LOCK, back to acquiring\n";
    }
    else if (previous == ChannelState::Confirming && m_state == ChannelState::Tracking)
    {
        m_pendingStateMessage += "SV ID " + std::to_string(svId) + " tracking CONFIRMED\n";
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
        const size_t currentReadSample = phase + (m_navBitOffset * GPS_NAV_CODE_PERIODS_PER_BIT);
        const size_t anchorSample = std::min(m_lastSubframeStartSample, currentReadSample);
        const size_t margin = phase + 40;
        if (anchorSample > margin)
        {
            dropSamples = ((anchorSample - margin) / GPS_NAV_CODE_PERIODS_PER_BIT) * GPS_NAV_CODE_PERIODS_PER_BIT;
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
            dropSamples = (minSearchedBits - 2) * GPS_NAV_CODE_PERIODS_PER_BIT;
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

    const size_t dropBits = dropSamples / GPS_NAV_CODE_PERIODS_PER_BIT;
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
    if (!decoder.processPromptSignal(svId,
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

    mergeSubframe(ephem, m_accumulatedEphemeris, m_seenSubframeMask);

    m_lastSubframeTow = ephem.tow;
    m_lastSubframeStartSample = subframeStartSample;
    m_accumulatedEphemeris.isValid = hasCompleteEphemeris();

    return hasCompleteEphemeris();
}

void Channel::mergeSubframe(const GpsEphemeris &decoded, GpsEphemeris &accumulated, uint8_t &seenSubframeMask)
{
    accumulated.svId = decoded.svId;
    accumulated.tow = decoded.tow;
    accumulated.subframeId = decoded.subframeId;

    switch (decoded.subframeId)
    {
        case 1:
            accumulated.weekNumber = decoded.weekNumber;
            accumulated.iodc = decoded.iodc;
            accumulated.toc = decoded.toc;
            accumulated.af0 = decoded.af0;
            accumulated.af1 = decoded.af1;
            accumulated.af2 = decoded.af2;
            accumulated.tgd = decoded.tgd;
            seenSubframeMask |= 0x1;
            break;
        case 2:
            accumulated.iode2 = decoded.iode2;
            accumulated.toe = decoded.toe;
            accumulated.sqrtA = decoded.sqrtA;
            accumulated.e = decoded.e;
            accumulated.M0 = decoded.M0;
            accumulated.deltaN = decoded.deltaN;
            accumulated.Cuc = decoded.Cuc;
            accumulated.Cus = decoded.Cus;
            accumulated.Crs = decoded.Crs;
            seenSubframeMask |= 0x2;
            break;
        case 3:
            accumulated.iode3 = decoded.iode3;
            accumulated.i0 = decoded.i0;
            accumulated.idot = decoded.idot;
            accumulated.omega0 = decoded.omega0;
            accumulated.omegaDot = decoded.omegaDot;
            accumulated.omega = decoded.omega;
            accumulated.Cic = decoded.Cic;
            accumulated.Cis = decoded.Cis;
            accumulated.Crc = decoded.Crc;
            seenSubframeMask |= 0x4;
            break;
        default:
            return;
    }

    const int iodcLow8 = accumulated.iodc & 0xFF;
    if (decoded.subframeId == 1)
    {
        if ((seenSubframeMask & 0x2) != 0 && accumulated.iode2 != iodcLow8)
        {
            seenSubframeMask &= static_cast<uint8_t>(~0x2);
        }
        if ((seenSubframeMask & 0x4) != 0 && accumulated.iode3 != iodcLow8)
        {
            seenSubframeMask &= static_cast<uint8_t>(~0x4);
        }
    }
    else if (decoded.subframeId == 2)
    {
        if ((seenSubframeMask & 0x1) != 0 && accumulated.iode2 != iodcLow8)
        {
            seenSubframeMask &= static_cast<uint8_t>(~0x1);
        }
        if ((seenSubframeMask & 0x4) != 0 && accumulated.iode3 != accumulated.iode2)
        {
            seenSubframeMask &= static_cast<uint8_t>(~0x4);
        }
    }
    else if (decoded.subframeId == 3)
    {
        if ((seenSubframeMask & 0x1) != 0 && accumulated.iode3 != iodcLow8)
        {
            seenSubframeMask &= static_cast<uint8_t>(~0x1);
        }
        if ((seenSubframeMask & 0x2) != 0 && accumulated.iode2 != accumulated.iode3)
        {
            seenSubframeMask &= static_cast<uint8_t>(~0x2);
        }
    }
}

bool Channel::hasCompleteEphemeris() const
{
    return (m_seenSubframeMask & 0x7) == 0x7;
}
