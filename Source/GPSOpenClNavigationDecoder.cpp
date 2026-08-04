#include "GPSOpenClNavigationDecoder.h"

#include <algorithm>
#include <array>
#include <cmath>

using namespace GPSOpenCl;

NavigationDecoder::NavigationDecoder() : m_inputConfig{STRUCT_VERSION_1, 0x1F}
{
}

NavigationDecoder::NavigationDecoder(const NavDecoderInput &input) : m_inputConfig(input)
{
}

NavigationDecoder::~NavigationDecoder() = default;

NavDecoderOutput NavigationDecoder::ephemerisToOutput(const GpsEphemeris &ephem)
{
    NavDecoderOutput out{};
    out.structVersion = STRUCT_VERSION_1;
    out.svId = ephem.svId;
    out.weekNumber = ephem.weekNumber;
    out.tow = ephem.tow;
    out.subframeId = ephem.subframeId;
    out.isValid = ephem.isValid ? 1 : 0;
    out.toc = ephem.toc;
    out.af0 = ephem.af0;
    out.af1 = ephem.af1;
    out.af2 = ephem.af2;
    out.tgd = ephem.tgd;
    out.toe = ephem.toe;
    out.sqrtA = ephem.sqrtA;
    out.e = ephem.e;
    out.i0 = ephem.i0;
    out.omega0 = ephem.omega0;
    out.omega = ephem.omega;
    out.M0 = ephem.M0;
    out.deltaN = ephem.deltaN;
    out.omegaDot = ephem.omegaDot;
    out.idot = ephem.idot;
    out.Cuc = ephem.Cuc;
    out.Cus = ephem.Cus;
    out.Crc = ephem.Crc;
    out.Crs = ephem.Crs;
    out.Cic = ephem.Cic;
    out.Cis = ephem.Cis;
    return out;
}

GpsEphemeris NavigationDecoder::outputToEphemeris(const NavDecoderOutput &out)
{
    GpsEphemeris ephem{};
    ephem.svId = out.svId;
    ephem.weekNumber = out.weekNumber;
    ephem.tow = out.tow;
    ephem.subframeId = out.subframeId;
    ephem.isValid = (out.isValid != 0);
    ephem.toc = out.toc;
    ephem.af0 = out.af0;
    ephem.af1 = out.af1;
    ephem.af2 = out.af2;
    ephem.tgd = out.tgd;
    ephem.toe = out.toe;
    ephem.sqrtA = out.sqrtA;
    ephem.e = out.e;
    ephem.i0 = out.i0;
    ephem.omega0 = out.omega0;
    ephem.omega = out.omega;
    ephem.M0 = out.M0;
    ephem.deltaN = out.deltaN;
    ephem.omegaDot = out.omegaDot;
    ephem.idot = out.idot;
    ephem.Cuc = out.Cuc;
    ephem.Cus = out.Cus;
    ephem.Crc = out.Crc;
    ephem.Crs = out.Crs;
    ephem.Cic = out.Cic;
    ephem.Cis = out.Cis;
    return ephem;
}

bool NavigationDecoder::decodeSubframe(const std::vector<uint32_t> &words30bit, NavDecoderOutput &output)
{
    GpsEphemeris ephem{};
    const bool res = decodeSubframe(words30bit, ephem);
    if (res)
    {
        output = ephemerisToOutput(ephem);
    }
    return res;
}

bool NavigationDecoder::findPreamble(const std::vector<bool> &bits, size_t &preambleIndex, bool &inverted)
{

    const uint8_t PREAMBLE_NORM = 0x8B;
    const uint8_t PREAMBLE_INV = 0x74;

    if (bits.size() < 300)
    {
        return false;
    }

    for (size_t i = 0; i <= bits.size() - 300; i++)
    {
        uint8_t byteVal = 0;
        for (int b = 0; b < 8; b++)
        {
            byteVal = (byteVal << 1) | (bits[i + b] ? 1 : 0);
        }

        if (byteVal == PREAMBLE_NORM)
        {
            preambleIndex = i;
            inverted = false;
            return true;
        }
        if (byteVal == PREAMBLE_INV)
        {
            preambleIndex = i;
            inverted = true;
            return true;
        }
    }

    return false;
}

int32_t NavigationDecoder::extractSignedBits(uint32_t val, int startBitFromMSB, int numBits)
{
    const uint32_t uval = extractUnsignedBits(val, startBitFromMSB, numBits);
    if ((uval & (1u << (numBits - 1))) != 0u)
    {
        const auto mask = static_cast<int32_t>((1u << numBits) - 1);
        return static_cast<int32_t>(uval | ~mask);
    }
    return static_cast<int32_t>(uval);
}

uint32_t NavigationDecoder::extractUnsignedBits(uint32_t val, int startBitFromMSB, int numBits)
{

    int shiftRight = 30 - (startBitFromMSB + numBits - 1);
    shiftRight = std::max(shiftRight, 0);
    const uint32_t mask = (1u << numBits) - 1;
    return (val >> shiftRight) & mask;
}

bool NavigationDecoder::checkParity(uint32_t word30bit, bool prevD29, bool prevD30)
{

    bool d[25];
    for (int i = 1; i <= 24; i++)
    {

        d[i] = ((word30bit >> (30 - i)) & 1) != 0;
    }

    const bool D25 =
        ((prevD29 ^ d[1] ^ static_cast<int>(d[2]) ^ static_cast<int>(d[3]) ^ static_cast<int>(d[5]) ^
          static_cast<int>(d[6]) ^ static_cast<int>(d[10]) ^ static_cast<int>(d[11]) ^ static_cast<int>(d[12]) ^
          static_cast<int>(d[13]) ^ static_cast<int>(d[14]) ^ static_cast<int>(d[17]) ^ static_cast<int>(d[18]) ^
          static_cast<int>(d[20]) ^ static_cast<int>(d[23])) != 0);
    const bool D26 =
        ((prevD30 ^ d[2] ^ static_cast<int>(d[3]) ^ static_cast<int>(d[4]) ^ static_cast<int>(d[6]) ^
          static_cast<int>(d[7]) ^ static_cast<int>(d[11]) ^ static_cast<int>(d[12]) ^ static_cast<int>(d[13]) ^
          static_cast<int>(d[14]) ^ static_cast<int>(d[15]) ^ static_cast<int>(d[18]) ^ static_cast<int>(d[19]) ^
          static_cast<int>(d[21]) ^ static_cast<int>(d[24])) != 0);
    const bool D27 =
        ((prevD29 ^ d[1] ^ static_cast<int>(d[3]) ^ static_cast<int>(d[4]) ^ static_cast<int>(d[5]) ^
          static_cast<int>(d[7]) ^ static_cast<int>(d[8]) ^ static_cast<int>(d[12]) ^ static_cast<int>(d[13]) ^
          static_cast<int>(d[14]) ^ static_cast<int>(d[15]) ^ static_cast<int>(d[16]) ^ static_cast<int>(d[19]) ^
          static_cast<int>(d[20]) ^ static_cast<int>(d[22])) != 0);
    const bool D28 =
        ((prevD30 ^ d[2] ^ static_cast<int>(d[4]) ^ static_cast<int>(d[5]) ^ static_cast<int>(d[6]) ^
          static_cast<int>(d[8]) ^ static_cast<int>(d[9]) ^ static_cast<int>(d[13]) ^ static_cast<int>(d[14]) ^
          static_cast<int>(d[15]) ^ static_cast<int>(d[16]) ^ static_cast<int>(d[17]) ^ static_cast<int>(d[20]) ^
          static_cast<int>(d[21]) ^ static_cast<int>(d[23])) != 0);
    const bool D29 =
        ((prevD30 ^ d[1] ^ static_cast<int>(d[3]) ^ static_cast<int>(d[5]) ^ static_cast<int>(d[6]) ^
          static_cast<int>(d[7]) ^ static_cast<int>(d[9]) ^ static_cast<int>(d[10]) ^ static_cast<int>(d[14]) ^
          static_cast<int>(d[15]) ^ static_cast<int>(d[16]) ^ static_cast<int>(d[17]) ^ static_cast<int>(d[18]) ^
          static_cast<int>(d[21]) ^ static_cast<int>(d[22]) ^ static_cast<int>(d[24])) != 0);
    const bool D30 = ((prevD29 ^ d[3] ^ static_cast<int>(d[5]) ^ static_cast<int>(d[6]) ^ static_cast<int>(d[8]) ^
                       static_cast<int>(d[9]) ^ static_cast<int>(d[10]) ^ static_cast<int>(d[11]) ^
                       static_cast<int>(d[13]) ^ static_cast<int>(d[15]) ^ static_cast<int>(d[19]) ^
                       static_cast<int>(d[22]) ^ static_cast<int>(d[23]) ^ static_cast<int>(d[24])) != 0);

    const bool p25 = ((word30bit >> 5) & 1) != 0;
    const bool p26 = ((word30bit >> 4) & 1) != 0;
    const bool p27 = ((word30bit >> 3) & 1) != 0;
    const bool p28 = ((word30bit >> 2) & 1) != 0;
    const bool p29 = ((word30bit >> 1) & 1) != 0;
    const bool p30 = (word30bit & 1) != 0;

    return (D25 == p25) && (D26 == p26) && (D27 == p27) && (D28 == p28) && (D29 == p29) && (D30 == p30);
}

std::vector<bool> NavigationDecoder::promptToBits(const ComplexFloatVector &promptHistory)
{
    std::vector<bool> bits;
    bits.reserve(promptHistory.size() / 20);

    for (size_t i = 0; i + 20 <= promptHistory.size(); i += 20)
    {
        float sumRe = 0.0f;
        for (size_t k = 0; k < 20; k++)
        {
            sumRe += promptHistory[i + k].real();
        }
        bits.push_back(sumRe > 0.0f);
    }
    return bits;
}

bool NavigationDecoder::decodeSubframe(const std::vector<uint32_t> &words30bit, GpsEphemeris &ephem)
{
    if (words30bit.size() < 10)
    {
        return false;
    }

    const uint32_t howWord = words30bit[1];
    const uint32_t towCount = extractUnsignedBits(howWord, 1, 17);
    ephem.subframeId = static_cast<int>(extractUnsignedBits(howWord, 20, 3));
    ephem.tow = towCount * 6.0;

    if (ephem.subframeId < 1 || ephem.subframeId > 5 ||
        (((m_inputConfig.subframeSearchMask >> (ephem.subframeId - 1)) & 0x1u) == 0u))
    {

        ephem.isValid = false;
        return false;
    }

    const uint32_t w3 = words30bit[2];
    const uint32_t w4 = words30bit[3];
    const uint32_t w5 = words30bit[4];
    const uint32_t w6 = words30bit[5];
    const uint32_t w7 = words30bit[6];
    const uint32_t w8 = words30bit[7];
    const uint32_t w9 = words30bit[8];
    const uint32_t w10 = words30bit[9];

    if (ephem.subframeId == 1)
    {

        ephem.weekNumber = static_cast<int>(extractUnsignedBits(w3, 1, 10));
        ephem.iodc = static_cast<int>((extractUnsignedBits(w3, 23, 2) << 8) | extractUnsignedBits(w8, 1, 8));
        ephem.tgd = extractSignedBits(w7, 17, 8) * std::pow(2.0, -31);
        ephem.toc = extractUnsignedBits(w8, 9, 16) * std::pow(2.0, 4);
        ephem.af2 = extractSignedBits(w9, 1, 8) * std::pow(2.0, -55);
        ephem.af1 = extractSignedBits(w9, 9, 16) * std::pow(2.0, -43);
        ephem.af0 = extractSignedBits(w10, 1, 22) * std::pow(2.0, -31);
        ephem.isValid = true;
    }
    else if (ephem.subframeId == 2)
    {

        ephem.Crs = extractSignedBits(w3, 9, 16) * std::pow(2.0, -5);
        ephem.deltaN = extractSignedBits(w4, 1, 16) * std::pow(2.0, -43) * M_PI;
        const uint32_t m0RawU =
            (static_cast<uint32_t>(extractSignedBits(w4, 17, 8)) << 24) | extractUnsignedBits(w5, 1, 24);
        auto m0Raw = static_cast<int32_t>(m0RawU);
        ephem.M0 = m0Raw * std::pow(2.0, -31) * M_PI;
        ephem.Cuc = extractSignedBits(w6, 1, 16) * std::pow(2.0, -29);
        const uint32_t eRaw = (extractUnsignedBits(w6, 17, 8) << 24) | extractUnsignedBits(w7, 1, 24);
        ephem.e = eRaw * std::pow(2.0, -33);
        ephem.Cus = extractSignedBits(w8, 1, 16) * std::pow(2.0, -29);
        const uint32_t sqrtARaw = (extractUnsignedBits(w8, 17, 8) << 24) | extractUnsignedBits(w9, 1, 24);
        ephem.sqrtA = sqrtARaw * std::pow(2.0, -19);
        ephem.toe = extractUnsignedBits(w10, 1, 16) * std::pow(2.0, 4);
        ephem.iode2 = static_cast<int>(extractUnsignedBits(w3, 1, 8));
        ephem.isValid = true;
    }
    else if (ephem.subframeId == 3)
    {

        ephem.Cic = extractSignedBits(w3, 1, 16) * std::pow(2.0, -29);
        const uint32_t omega0RawU =
            (static_cast<uint32_t>(extractSignedBits(w3, 17, 8)) << 24) | extractUnsignedBits(w4, 1, 24);
        auto omega0Raw = static_cast<int32_t>(omega0RawU);
        ephem.omega0 = omega0Raw * std::pow(2.0, -31) * M_PI;
        ephem.Cis = extractSignedBits(w5, 1, 16) * std::pow(2.0, -29);
        const uint32_t i0RawU =
            (static_cast<uint32_t>(extractSignedBits(w5, 17, 8)) << 24) | extractUnsignedBits(w6, 1, 24);
        auto i0Raw = static_cast<int32_t>(i0RawU);
        ephem.i0 = i0Raw * std::pow(2.0, -31) * M_PI;
        ephem.Crc = extractSignedBits(w7, 1, 16) * std::pow(2.0, -5);
        const uint32_t omegaRawU =
            (static_cast<uint32_t>(extractSignedBits(w7, 17, 8)) << 24) | extractUnsignedBits(w8, 1, 24);
        auto omegaRaw = static_cast<int32_t>(omegaRawU);
        ephem.omega = omegaRaw * std::pow(2.0, -31) * M_PI;
        ephem.omegaDot = extractSignedBits(w9, 1, 24) * std::pow(2.0, -43) * M_PI;
        ephem.iode3 = static_cast<int>(extractUnsignedBits(w10, 1, 8));
        ephem.idot = extractSignedBits(w10, 9, 14) * std::pow(2.0, -43) * M_PI;
        ephem.isValid = true;
    }
    else if (ephem.subframeId == 4)
    {
        decodeIonosphericParams(words30bit);
    }

    if (!ephem.isValid)
    {
        return false;
    }

    if (ephem.subframeId == 2 && ephem.iodc != 0)
    {
        const int iodcLow8 = ephem.iodc & 0xFF;
        if (ephem.iode2 != iodcLow8)
        {
            ephem.isValid = false;
            return false;
        }
    }
    if (ephem.subframeId == 3 && ephem.iodc != 0)
    {
        const int iodcLow8 = ephem.iodc & 0xFF;
        if (ephem.iode3 != iodcLow8)
        {
            ephem.isValid = false;
            return false;
        }
    }

    return ephem.isValid;
}

void NavigationDecoder::decodeIonosphericParams(const std::vector<uint32_t> &words30bit)
{
    const uint32_t w3 = words30bit[2];
    const uint32_t dataId = extractUnsignedBits(w3, 1, 2);
    const uint32_t pageId = extractUnsignedBits(w3, 3, 6);
    if (dataId != 1 || pageId != 56)
    {
        return;
    }

    const uint32_t w4 = words30bit[3];
    const uint32_t w5 = words30bit[4];

    m_ionoParams.structVersion = STRUCT_VERSION_1;
    m_ionoParams.alpha0 = extractSignedBits(w3, 9, 8) * std::pow(2.0, -30);
    m_ionoParams.alpha1 = extractSignedBits(w3, 17, 8) * std::pow(2.0, -27);
    m_ionoParams.alpha2 = extractSignedBits(w4, 1, 8) * std::pow(2.0, -24);
    m_ionoParams.alpha3 = extractSignedBits(w4, 9, 8) * std::pow(2.0, -24);
    m_ionoParams.beta0 = extractSignedBits(w4, 17, 8) * std::pow(2.0, 11);
    m_ionoParams.beta1 = extractSignedBits(w5, 1, 8) * std::pow(2.0, 14);
    m_ionoParams.beta2 = extractSignedBits(w5, 9, 8) * std::pow(2.0, 16);
    m_ionoParams.beta3 = extractSignedBits(w5, 17, 8) * std::pow(2.0, 16);
    m_hasIonoParams = true;
}

bool NavigationDecoder::decodeAtPhaseOffset(int svId,
                                            const ComplexFloatVector &promptHistory,
                                            int phase,
                                            size_t &bitOffset,
                                            GpsEphemeris &ephem,
                                            size_t &subframeStartSample,
                                            const FloatVector *codePhaseHistory)
{
    bool hadEnoughData = false;
    const bool decoded = tryDecodeAtBitPosition(
        svId, promptHistory, phase, bitOffset, hadEnoughData, ephem, subframeStartSample, codePhaseHistory);

    if (decoded)
    {
        bitOffset += 300;
    }
    else if (hadEnoughData)
    {
        bitOffset++;
    }

    return decoded;
}

bool NavigationDecoder::tryDecodeAtBitPosition(int svId,
                                               const ComplexFloatVector &promptHistory,
                                               int phase,
                                               size_t bitPosition,
                                               bool &hadEnoughData,
                                               GpsEphemeris &ephem,
                                               size_t &subframeStartSample,
                                               const FloatVector *codePhaseHistory)
{
    hadEnoughData = false;

    auto phaseBase = static_cast<size_t>(phase);
    if (phaseBase > promptHistory.size())
    {
        return false;
    }

    const size_t startSample = phaseBase + (bitPosition * 20);
    if (startSample + static_cast<size_t>(300 * 20) > promptHistory.size())
    {
        return false;
    }

    hadEnoughData = true;

    auto demodulateBit = [&](size_t bitIndex) -> bool
    {
        float sumRe = 0.0f;
        const size_t base = startSample + (bitIndex * 20);
        for (int k = 0; k < 20; k++)
        {
            sumRe += promptHistory[base + k].real();
        }
        return sumRe > 0.0f;
    };

    std::array<bool, 300> bits{};
    for (size_t i = 0; i < 8; i++)
    {
        bits[i] = demodulateBit(i);
    }

    uint8_t byteVal = 0;
    for (size_t b = 0; b < 8; b++)
    {
        byteVal = (byteVal << 1) | (bits[b] ? 1u : 0u);
    }

    bool inverted = false;
    if (byteVal == 0x8B)
    {
        inverted = false;
    }
    else if (byteVal == 0x74)
    {
        inverted = true;
    }
    else
    {
        return false;
    }

    for (size_t i = 8; i < 300; i++)
    {
        bits[i] = demodulateBit(i);
    }

    auto bitAt = [&](size_t idx) -> bool
    {
        const bool v = bits[idx];
        return inverted ? !v : v;
    };

    bool prevD29 = false;
    bool prevD30 = false;
    if (bitPosition >= 2)
    {
        for (int back = 2; back >= 1; back--)
        {
            const size_t base = startSample - (static_cast<size_t>(back) * 20);
            float sumRe = 0.0f;
            for (int k = 0; k < 20; k++)
            {
                sumRe += promptHistory[base + k].real();
            }
            const bool bit = sumRe > 0.0f;
            const bool v = inverted ? !bit : bit;
            if (back == 2)
            {
                prevD29 = v;
            }
            else
            {
                prevD30 = v;
            }
        }
    }

    m_wordsScratch.assign(10, 0);
    std::vector<uint32_t> &words = m_wordsScratch;
    for (int w = 0; w < 10; w++)
    {
        const size_t wordStartBit = static_cast<size_t>(w) * 30;
        uint32_t raw = 0;
        for (int b = 0; b < 30; b++)
        {
            raw = (raw << 1) | (bitAt(wordStartBit + static_cast<size_t>(b)) ? 1u : 0u);
        }

        const uint32_t dataBitsMask = 0x3F'FF'FF'C0u;
        const uint32_t dataWord = prevD30 ? (raw ^ dataBitsMask) : raw;

        if (!checkParity(dataWord, prevD29, prevD30))
        {
            return false;
        }

        words[static_cast<size_t>(w)] = dataWord;

        prevD29 = ((raw >> 1) & 1u) != 0;
        prevD30 = (raw & 1u) != 0;
    }

    // The first bit-sync phase whose preamble and parity pass can sit a whole block away from the
    // true bit edge (19 of 20 blocks per bit still integrate correctly), which would shift every
    // derived transmit time by that block. The decoded bits are parity-verified, so a matched
    // filter over candidate start blocks pins the block containing the true edge: each bit's first
    // block carries mixed adjacent-bit signal split at the sub-block edge position known from the
    // DLL code phase, and modeling that split keeps adjacent candidates distinguishable by a full
    // block of energy per bit transition wherever the edge sits inside the block.
    double edgeFraction = 0.5;
    if (codePhaseHistory != nullptr && startSample < codePhaseHistory->size())
    {
        const double phaseChips = static_cast<double>((*codePhaseHistory)[startSample]);
        edgeFraction = (1023.0 - phaseChips) / 1023.0;
        edgeFraction = std::min(std::max(edgeFraction, 0.0), 1.0);
    }

    std::ptrdiff_t bestOffset = 0;
    double bestMetric = -1.0;
    for (std::ptrdiff_t candidate = -2; candidate <= 2; candidate++)
    {
        if (candidate < 0 && startSample < static_cast<size_t>(-candidate))
        {
            continue;
        }
        const size_t candidateStart = startSample + candidate;
        if (candidateStart + static_cast<size_t>(300 * 20) > promptHistory.size())
        {
            continue;
        }

        double metric = 0.0;
        for (size_t i = 0; i < 300; i++)
        {
            const size_t base = candidateStart + (i * 20);
            const double sign = bits[i] ? 1.0 : -1.0;

            float sumRe = 0.0f;
            for (int k = 1; k < 20; k++)
            {
                sumRe += promptHistory[base + k].real();
            }
            metric += sign * sumRe;

            if (i > 0)
            {
                const double previousSign = bits[i - 1] ? 1.0 : -1.0;
                const double boundaryWeight = (edgeFraction * previousSign) + ((1.0 - edgeFraction) * sign);
                metric += boundaryWeight * static_cast<double>(promptHistory[base].real());
            }
        }
        metric = std::fabs(metric);

        if (metric > bestMetric)
        {
            bestMetric = metric;
            bestOffset = candidate;
        }
    }

    subframeStartSample = startSample + bestOffset;
    ephem.svId = svId;
    return decodeSubframe(words, ephem);
}

bool NavigationDecoder::processPromptSignal(int svId,
                                            const ComplexFloatVector &promptHistory,
                                            int &bitSyncPhase,
                                            std::vector<size_t> &searchPositions,
                                            size_t &bitOffset,
                                            GpsEphemeris &ephem,
                                            size_t &subframeStartSample)
{
    ephem.svId = svId;
    ephem.isValid = false;

    bool decoded = false;

    if (bitSyncPhase >= 0)
    {
        decoded = decodeAtPhaseOffset(svId, promptHistory, bitSyncPhase, bitOffset, ephem, subframeStartSample);
    }
    else
    {
        if (searchPositions.size() != 20)
        {
            searchPositions.assign(20, 0);
        }

        for (int phase = 0; phase < 20 && !decoded; phase++)
        {
            bool hadEnoughData = false;
            decoded = tryDecodeAtBitPosition(svId,
                                             promptHistory,
                                             phase,
                                             searchPositions[static_cast<size_t>(phase)],
                                             hadEnoughData,
                                             ephem,
                                             subframeStartSample);
            if (decoded)
            {
                bitSyncPhase = phase;
                bitOffset = searchPositions[static_cast<size_t>(phase)] + 300;
            }
            else if (hadEnoughData)
            {
                searchPositions[static_cast<size_t>(phase)]++;
            }
        }
    }

    if (decoded && m_sink)
    {
        m_sink->publishNavDecoderOutput(ephemerisToOutput(ephem));
    }
    return decoded;
}

bool NavigationDecoder::processPromptSignal(int svId,
                                            const ComplexFloatVector &promptHistory,
                                            int &bitSyncPhase,
                                            std::vector<size_t> &searchPositions,
                                            size_t &bitOffset,
                                            NavDecoderOutput &output,
                                            size_t &subframeStartSample)
{
    GpsEphemeris ephem{};
    const bool res =
        processPromptSignal(svId, promptHistory, bitSyncPhase, searchPositions, bitOffset, ephem, subframeStartSample);
    if (res)
    {
        output = ephemerisToOutput(ephem);
    }
    return res;
}
