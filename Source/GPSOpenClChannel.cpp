#include "GPSOpenClChannel.h"
#include "GPSOpenClCommon.h"

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace GPSOpenCl;

Channel::Channel()
    : m_svId(0),
      m_acquisitionPeakIndex(0),
      m_acquisitionPeakValue(0.0f),
      m_acquisitionPeakFrequency(0.0f),
      m_acquisitionMeanValue(0.0f),
      m_acquisitionCN0(0.0f),
      m_acquisitionPeakRatio(0.0f),
      m_acquisitionProcessingGain(10.0 * std::log10(GPS_CA_CODE_FREQUENCY_HZ / GPS_CA_CODE_LENGTH)),
      m_isAcquired(false),
      m_tracking(nullptr),
      m_navBitOffset(0),
      m_seenSubframeMask(0),
      m_accumulatedEphemeris(),
      m_lastSubframeTow(0.0),
      m_lastSubframeStartSample(0),
      m_state(ChannelState::Acquiring),
      m_confirmProgress(0),
      m_lossProgress(0),
      m_blocksInConfirming(0),
      m_carrierLockThreshold(0.5f),
      m_codeLockRatioTolerance(0.3f),
      m_confirmDebounceBlocks(50),
      m_confirmTimeoutBlocks(200),
      m_lossDebounceBlocks(100)
{
}

Channel::~Channel()
{
    if (m_tracking)
    {
        delete m_tracking;
        m_tracking = nullptr;
    }
}

void Channel::insertAcquisitionMetrics(float peakValue, int peakIndex, float peakFrequency, float meanValue)
{
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

void Channel::checkAcquisition()
{
    std::cout << "SV ID " << m_svId << " C/N0 : " << m_acquisitionCN0 << std::endl;
}

void Channel::getAcquisitionResults(int *peakIndex, float *peakValue, float *peakFrequency, float *meanValue,
                                    float *cno, float *peakRatio)
{
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
    m_sink = sink;
    if (m_tracking)
    {
        m_tracking->setSink(m_sink);
    }
}

void Channel::initTracking(const Settings::Configuration &conf, float dopplerHz, float codePhaseChips)
{
    if (!m_tracking)
    {
        m_tracking = new Tracking(conf);
        m_tracking->setSink(m_sink);
    }
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
    m_navBitOffset = 0;
    m_seenSubframeMask = 0;
    m_accumulatedEphemeris = GpsEphemeris();
    m_lastSubframeTow = 0.0;
    m_lastSubframeStartSample = 0;
}

void Channel::trackBlock(const ComplexFloatVector &input)
{
    if (m_tracking && m_isAcquired)
    {
        ComplexFloatVector *promptOutput = (m_state == ChannelState::Tracking) ? &m_promptHistory : nullptr;
        m_tracking->doWork(input, m_svId, promptOutput, static_cast<uint32_t>(m_state));
        evaluateLockState();
    }
}

ChannelState Channel::computeNextState(ChannelState current, bool goodBlock, int &confirmProgress, int &lossProgress,
                                       int &blocksInConfirming, int confirmDebounceBlocks, int confirmTimeoutBlocks,
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
    bool good = m_tracking->getCarrierLockIndicator() >= m_carrierLockThreshold &&
                std::fabs(m_tracking->getCodeLockRatio() - 1.0f) <= m_codeLockRatioTolerance;

    ChannelState previous = m_state;
    m_state = computeNextState(m_state, good, m_confirmProgress, m_lossProgress, m_blocksInConfirming,
                               m_confirmDebounceBlocks, m_confirmTimeoutBlocks, m_lossDebounceBlocks);

    if (previous == ChannelState::Confirming && m_state == ChannelState::Acquiring)
    {
        m_isAcquired = false;
        resetNavigationState();
        std::cout << "SV ID " << m_svId << " confirmation TIMED OUT (lock: carrier=" << m_tracking->getCarrierLockIndicator()
                  << " code=" << m_tracking->getCodeLockRatio() << "), back to acquiring" << std::endl;
    }
    else if (previous == ChannelState::Tracking && m_state == ChannelState::Acquiring)
    {
        m_isAcquired = false;
        resetNavigationState();
        std::cout << "SV ID " << m_svId << " LOST LOCK, back to acquiring" << std::endl;
    }
    else if (previous == ChannelState::Confirming && m_state == ChannelState::Tracking)
    {
        std::cout << "SV ID " << m_svId << " tracking CONFIRMED" << std::endl;
    }
}

bool Channel::updateNavigation(NavigationDecoder &decoder)
{
    GpsEphemeris ephem = GpsEphemeris();
    size_t subframeStartSample = 0;
    if (!decoder.processPromptSignal(m_svId, m_promptHistory, m_navBitOffset, ephem, subframeStartSample))
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
