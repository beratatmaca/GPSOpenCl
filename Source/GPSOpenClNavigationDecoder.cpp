#include "GPSOpenClNavigationDecoder.h"

#include <cmath>
#include <iostream>

using namespace GPSOpenCl;

NavigationDecoder::NavigationDecoder()
    : m_inputConfig{STRUCT_VERSION_1, 0x1F}
{
}

NavigationDecoder::NavigationDecoder(const NavDecoderInput &input)
    : m_inputConfig(input)
{
}

NavigationDecoder::~NavigationDecoder()
{
}

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
    bool res = decodeSubframe(words30bit, ephem);
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
        else if (byteVal == PREAMBLE_INV)
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
    uint32_t uval = extractUnsignedBits(val, startBitFromMSB, numBits);
    if (uval & (1u << (numBits - 1)))
    {
        int32_t mask = (1u << numBits) - 1;
        return static_cast<int32_t>(uval | ~mask);
    }
    return static_cast<int32_t>(uval);
}

uint32_t NavigationDecoder::extractUnsignedBits(uint32_t val, int startBitFromMSB, int numBits)
{

    int shiftRight = 30 - (startBitFromMSB + numBits - 1);
    if (shiftRight < 0) shiftRight = 0;
    uint32_t mask = (1u << numBits) - 1;
    return (val >> shiftRight) & mask;
}

bool NavigationDecoder::checkParity(uint32_t word30bit, bool prevD29, bool prevD30)
{

    bool d[25];
    for (int i = 1; i <= 24; i++)
    {

        d[i] = ((word30bit >> (30 - i)) & 1) != 0;
    }

    bool D25 = prevD29 ^ d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[17] ^ d[18] ^ d[20] ^ d[23];
    bool D26 = prevD30 ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[18] ^ d[19] ^ d[21] ^ d[24];
    bool D27 = prevD29 ^ d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[19] ^ d[20] ^ d[22];
    bool D28 = prevD30 ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[20] ^ d[21] ^ d[23];
    bool D29 = prevD30 ^ d[1] ^ d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[9] ^ d[10] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[21] ^ d[22] ^ d[24];
    bool D30 = prevD29 ^ d[3] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[10] ^ d[11] ^ d[13] ^ d[15] ^ d[19] ^ d[22] ^ d[23] ^ d[24];

    bool p25 = ((word30bit >> 5) & 1) != 0;
    bool p26 = ((word30bit >> 4) & 1) != 0;
    bool p27 = ((word30bit >> 3) & 1) != 0;
    bool p28 = ((word30bit >> 2) & 1) != 0;
    bool p29 = ((word30bit >> 1) & 1) != 0;
    bool p30 = (word30bit & 1) != 0;

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
    if (words30bit.size() < 10) return false;


    uint32_t howWord = words30bit[1];
    uint32_t towCount = extractUnsignedBits(howWord, 1, 17);
    ephem.subframeId = extractUnsignedBits(howWord, 20, 3);
    ephem.tow = towCount * 6.0;

    if (ephem.subframeId < 1 || ephem.subframeId > 5 ||
        !((m_inputConfig.subframeSearchMask >> (ephem.subframeId - 1)) & 0x1u))
    {

        ephem.isValid = false;
        return false;
    }

    uint32_t w3 = words30bit[2];
    uint32_t w4 = words30bit[3];
    uint32_t w5 = words30bit[4];
    uint32_t w6 = words30bit[5];
    uint32_t w7 = words30bit[6];
    uint32_t w8 = words30bit[7];
    uint32_t w9 = words30bit[8];
    uint32_t w10 = words30bit[9];

    if (ephem.subframeId == 1)
    {


        ephem.weekNumber = extractUnsignedBits(w3, 1, 10);
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
        int32_t m0Raw = (extractSignedBits(w4, 17, 8) << 24) | extractUnsignedBits(w5, 1, 24);
        ephem.M0 = m0Raw * std::pow(2.0, -31) * M_PI;
        ephem.Cuc = extractSignedBits(w6, 1, 16) * std::pow(2.0, -29);
        uint32_t eRaw = (extractUnsignedBits(w6, 17, 8) << 24) | extractUnsignedBits(w7, 1, 24);
        ephem.e = eRaw * std::pow(2.0, -33);
        ephem.Cus = extractSignedBits(w8, 1, 16) * std::pow(2.0, -29);
        uint32_t sqrtARaw = (extractUnsignedBits(w8, 17, 8) << 24) | extractUnsignedBits(w9, 1, 24);
        ephem.sqrtA = sqrtARaw * std::pow(2.0, -19);
        ephem.toe = extractUnsignedBits(w10, 1, 16) * std::pow(2.0, 4);
        ephem.isValid = true;
    }
    else if (ephem.subframeId == 3)
    {



        ephem.Cic = extractSignedBits(w3, 1, 16) * std::pow(2.0, -29);
        int32_t omega0Raw = (extractSignedBits(w3, 17, 8) << 24) | extractUnsignedBits(w4, 1, 24);
        ephem.omega0 = omega0Raw * std::pow(2.0, -31) * M_PI;
        ephem.Cis = extractSignedBits(w5, 1, 16) * std::pow(2.0, -29);
        int32_t i0Raw = (extractSignedBits(w5, 17, 8) << 24) | extractUnsignedBits(w6, 1, 24);
        ephem.i0 = i0Raw * std::pow(2.0, -31) * M_PI;
        ephem.Crc = extractSignedBits(w7, 1, 16) * std::pow(2.0, -5);
        int32_t omegaRaw = (extractSignedBits(w7, 17, 8) << 24) | extractUnsignedBits(w8, 1, 24);
        ephem.omega = omegaRaw * std::pow(2.0, -31) * M_PI;
        ephem.omegaDot = extractSignedBits(w9, 1, 24) * std::pow(2.0, -43) * M_PI;
        ephem.idot = extractSignedBits(w10, 9, 14) * std::pow(2.0, -43) * M_PI;
        ephem.isValid = true;
    }
    else if (ephem.subframeId == 4)
    {
        decodeIonosphericParams(words30bit);
    }

    return ephem.isValid;
}

void NavigationDecoder::decodeIonosphericParams(const std::vector<uint32_t> &words30bit)
{
    uint32_t w3 = words30bit[2];
    uint32_t dataId = extractUnsignedBits(w3, 1, 2);
    uint32_t pageId = extractUnsignedBits(w3, 3, 6);
    if (dataId != 1 || pageId != 56) return;

    uint32_t w4 = words30bit[3];
    uint32_t w5 = words30bit[4];

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

bool NavigationDecoder::processPromptSignal(int svId, const ComplexFloatVector &promptHistory, size_t &bitOffset,
                                            GpsEphemeris &ephem, size_t &subframeStartSample)
{
    ephem.svId = svId;
    ephem.isValid = false;

    std::vector<bool> allBits = promptToBits(promptHistory);
    if (bitOffset > allBits.size())
    {
        bitOffset = 0;
    }
    if (allBits.size() - bitOffset < 300)
    {
        return false;
    }

    std::vector<bool> searchBits(allBits.begin() + bitOffset, allBits.end());
    size_t relPreambleIdx = 0;
    bool inverted = false;
    if (!findPreamble(searchBits, relPreambleIdx, inverted))
    {
        return false;
    }

    size_t preambleIdx = bitOffset + relPreambleIdx;
    if (preambleIdx + 300 > allBits.size())
    {

        return false;
    }

    auto bitAt = [&](size_t idx) -> bool {
        bool v = allBits[idx];
        return inverted ? !v : v;
    };



    bool prevD29 = false;
    bool prevD30 = false;
    if (preambleIdx >= 2)
    {
        prevD29 = bitAt(preambleIdx - 2);
        prevD30 = bitAt(preambleIdx - 1);
    }

    std::vector<uint32_t> words(10, 0);
    for (int w = 0; w < 10; w++)
    {
        size_t wordStartBit = preambleIdx + static_cast<size_t>(w) * 30;
        uint32_t raw = 0;
        for (int b = 0; b < 30; b++)
        {
            raw = (raw << 1) | (bitAt(wordStartBit + b) ? 1u : 0u);
        }

        if (!checkParity(raw, prevD29, prevD30))
        {

            bitOffset = preambleIdx + 1;
            return false;
        }




        uint32_t dataWord = raw;
        if (prevD30)
        {
            const uint32_t dataBitsMask = 0x3FFFFFC0u;
            dataWord = raw ^ dataBitsMask;
        }
        words[w] = dataWord;

        prevD29 = ((raw >> 1) & 1u) != 0;
        prevD30 = (raw & 1u) != 0;
    }

    subframeStartSample = preambleIdx * 20;
    bitOffset = preambleIdx + 300;

    bool decoded = decodeSubframe(words, ephem);
    if (decoded && m_sink)
    {
        m_sink->publishNavDecoderOutput(ephemerisToOutput(ephem));
    }
    return decoded;
}

bool NavigationDecoder::processPromptSignal(int svId, const ComplexFloatVector &promptHistory, size_t &bitOffset,
                                           NavDecoderOutput &output, size_t &subframeStartSample)
{
    GpsEphemeris ephem{};
    bool res = processPromptSignal(svId, promptHistory, bitOffset, ephem, subframeStartSample);
    if (res)
    {
        output = ephemerisToOutput(ephem);
    }
    return res;
}
