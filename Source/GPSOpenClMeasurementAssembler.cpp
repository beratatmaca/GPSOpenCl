#include "GPSOpenClMeasurementAssembler.h"

#include "GPSOpenClCommon.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

using namespace GPSOpenCl;

bool MeasurementAssembler::computeElapsedSecondsSincePromptStart(size_t promptCount,
                                                                 size_t startSample,
                                                                 double &elapsedSecondsOut)
{
    if (promptCount < startSample)
    {
        return false;
    }
    elapsedSecondsOut = static_cast<double>(promptCount - startSample) * GPS_CA_CODE_PERIOD_SEC;
    return true;
}

double MeasurementAssembler::computeTransmitTime(double subframeStartTow,
                                                 double elapsedSeconds,
                                                 double driftChips,
                                                 double anchorChipsRaw)
{
    const double anchorChips = anchorChipsRaw - GPS_CA_CODE_LENGTH;
    return subframeStartTow + elapsedSeconds + ((driftChips + anchorChips) / GPS_CA_CODE_FREQUENCY_HZ);
}

void MeasurementAssembler::snapTransmitTimesToMedianArrival(std::vector<double> &transmitTimes,
                                                            const std::vector<double> &impliedArrivals)
{
    if (transmitTimes.empty() || transmitTimes.size() != impliedArrivals.size())
    {
        return;
    }

    std::vector<double> sortedArrivals = impliedArrivals;
    std::sort(sortedArrivals.begin(), sortedArrivals.end());
    const double medianArrival = sortedArrivals[sortedArrivals.size() / 2];

    constexpr double snapGuardBandCodePeriods = 0.25;
    for (size_t i = 0; i < transmitTimes.size(); i++)
    {
        const double offsetCodePeriods = (medianArrival - impliedArrivals[i]) / GPS_CA_CODE_PERIOD_SEC;
        const double wholePeriods = std::round(offsetCodePeriods);
        if (wholePeriods != 0.0 && std::fabs(offsetCodePeriods - wholePeriods) < snapGuardBandCodePeriods)
        {
            transmitTimes[i] += wholePeriods * GPS_CA_CODE_PERIOD_SEC;
        }
    }
}

bool MeasurementAssembler::assemble(const Channel *channels,
                                    int channelCount,
                                    const EcefPosition &referenceEcef,
                                    bool referenceTrusted,
                                    Measurements &out)
{
    out.ephemerides.clear();
    out.transmitTimesSec.clear();
    out.pseudorangesMeters.clear();
    out.prns.clear();
    out.receiverTimeSec = 0.0;

    for (int i = 0; i < channelCount; i++)
    {
        const Channel &channel = channels[i];
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

        const double maxAnchorAgeSeconds = 15.0;
        if (elapsedSeconds > maxAnchorAgeSeconds)
        {
            continue;
        }

        const double subframeStartTow = channel.getLastSubframeTow() - GPS_NAV_SUBFRAME_DURATION_SEC;

        const double driftChips = static_cast<double>(channel.getCumulativeDriftChipsAtSample(promptCount - 1)) -
            static_cast<double>(channel.getCumulativeDriftChipsAtSample(subframeStartSample));

        const auto anchorChipsRaw = static_cast<double>(channel.getCodePhaseAtSample(subframeStartSample));

        out.ephemerides.push_back(channel.getAccumulatedEphemeris());
        out.transmitTimesSec.push_back(
            computeTransmitTime(subframeStartTow, elapsedSeconds, driftChips, anchorChipsRaw));
        out.prns.push_back(channel.svId);
    }

    if (out.ephemerides.size() < 4)
    {
        return false;
    }

    if (referenceTrusted)
    {
        std::vector<double> impliedArrivals(out.transmitTimesSec.size());
        for (size_t i = 0; i < out.transmitTimesSec.size(); i++)
        {
            const SatelliteOrbit orbit = PVTSolver::computeSatelliteOrbit(out.ephemerides[i], out.transmitTimesSec[i]);
            const double dx = orbit.position.x - referenceEcef.x;
            const double dy = orbit.position.y - referenceEcef.y;
            const double dz = orbit.position.z - referenceEcef.z;
            impliedArrivals[i] = out.transmitTimesSec[i] - orbit.clockBias +
                (std::sqrt((dx * dx) + (dy * dy) + (dz * dz)) / SPEED_OF_LIGHT_M_S);
        }

        snapTransmitTimesToMedianArrival(out.transmitTimesSec, impliedArrivals);
    }

    out.receiverTimeSec = PVTSolver::computeReceiverTime(out.ephemerides, out.transmitTimesSec, referenceEcef);

    out.pseudorangesMeters.resize(out.transmitTimesSec.size());
    for (size_t i = 0; i < out.transmitTimesSec.size(); i++)
    {
        out.pseudorangesMeters[i] = (out.receiverTimeSec - out.transmitTimesSec[i]) * SPEED_OF_LIGHT_M_S;
    }

    return true;
}
