#include "GPSOpenClNavigationDecoder.h"

#include <cmath>
#include <iostream>

using namespace GPSOpenCl;

NavigationDecoder::NavigationDecoder()
{
}

NavigationDecoder::~NavigationDecoder()
{
}

bool NavigationDecoder::findPreamble(const std::vector<bool> &bits, size_t &preambleIndex, bool &inverted)
{
    // GPS L1 C/A Preamble byte: 0x8B (10001011) or inverted 0x74 (01110100)
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
    // startBitFromMSB is 1-indexed from MSB (bit 30 is LSB, bit 1 is MSB)
    int shiftRight = 30 - (startBitFromMSB + numBits - 1);
    if (shiftRight < 0) shiftRight = 0;
    uint32_t mask = (1u << numBits) - 1;
    return (val >> shiftRight) & mask;
}

bool NavigationDecoder::checkParity(uint32_t word30bit, bool prevD29, bool prevD30)
{
    // IS-GPS-200 30-bit word parity algorithm
    bool d[25];
    for (int i = 1; i <= 24; i++)
    {
        // bit 1 is MSB (bit index 29 in 0-indexed integer)
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

    // 1 bit = 20 ms (20 integration blocks of 1 ms)
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

    // Word 2: HOW Word
    uint32_t howWord = words30bit[1];
    uint32_t towCount = extractUnsignedBits(howWord, 1, 17);
    ephem.subframeId = extractUnsignedBits(howWord, 20, 3);
    ephem.tow = towCount * 6.0; // TOW in seconds

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
        ephem.weekNumber = extractUnsignedBits(w3, 3, 10);
        ephem.toc = extractUnsignedBits(w8, 22, 16) * std::pow(2.0, 4);
        ephem.af2 = extractSignedBits(w10, 3, 8) * std::pow(2.0, -55);
        ephem.af1 = extractSignedBits(w10, 11, 16) * std::pow(2.0, -43);
        int32_t af0_raw = (extractSignedBits(w10, 27, 2) << 20) | extractUnsignedBits(w9, 3, 20);
        ephem.af0 = af0_raw * std::pow(2.0, -31);
        ephem.isValid = true;
    }
    else if (ephem.subframeId == 2)
    {
        ephem.Crs = extractSignedBits(w3, 3, 16) * std::pow(2.0, -5);
        ephem.deltaN = extractSignedBits(w3, 19, 16) * std::pow(2.0, -43) * M_PI;
        ephem.M0 = ((extractSignedBits(w4, 3, 8) << 24) | extractUnsignedBits(w5, 3, 24)) * std::pow(2.0, -31) * M_PI;
        ephem.Cuc = extractSignedBits(w6, 3, 16) * std::pow(2.0, -29);
        ephem.e = ((extractUnsignedBits(w6, 19, 8) << 24) | extractUnsignedBits(w7, 3, 24)) * std::pow(2.0, -33);
        ephem.Cus = extractSignedBits(w8, 3, 16) * std::pow(2.0, -29);
        uint32_t sqrtA_raw = (extractUnsignedBits(w8, 19, 8) << 24) | extractUnsignedBits(w9, 3, 24);
        ephem.sqrtA = sqrtA_raw * std::pow(2.0, -19);
        ephem.toe = extractUnsignedBits(w10, 3, 16) * std::pow(2.0, 4);
        ephem.isValid = true;
    }
    else if (ephem.subframeId == 3)
    {
        ephem.Cic = extractSignedBits(w3, 3, 16) * std::pow(2.0, -29);
        uint32_t omega0_raw = (extractSignedBits(w3, 19, 8) << 24) | extractUnsignedBits(w4, 3, 24);
        ephem.omega0 = omega0_raw * std::pow(2.0, -31) * M_PI;
        ephem.Cis = extractSignedBits(w5, 3, 16) * std::pow(2.0, -29);
        uint32_t i0_raw = (extractSignedBits(w5, 19, 8) << 24) | extractUnsignedBits(w6, 3, 24);
        ephem.i0 = i0_raw * std::pow(2.0, -31) * M_PI;
        ephem.Crc = extractSignedBits(w7, 3, 16) * std::pow(2.0, -5);
        uint32_t omega_raw = (extractSignedBits(w7, 19, 8) << 24) | extractUnsignedBits(w8, 3, 24);
        ephem.omega = omega_raw * std::pow(2.0, -31) * M_PI;
        ephem.omegaDot = extractSignedBits(w9, 3, 24) * std::pow(2.0, -38) * M_PI;
        ephem.idot = extractSignedBits(w10, 11, 14) * std::pow(2.0, -43) * M_PI;
        ephem.isValid = true;
    }

    return ephem.isValid;
}

bool NavigationDecoder::processPromptSignal(int svId, const ComplexFloatVector &promptHistory, GpsEphemeris &ephem)
{
    ephem.svId = svId;
    ephem.isValid = false;

    std::vector<bool> bits = promptToBits(promptHistory);
    if (bits.size() < 300)
    {
        return false;
    }

    size_t preambleIdx = 0;
    bool inverted = false;
    if (!findPreamble(bits, preambleIdx, inverted))
    {
        return false;
    }

    std::vector<uint32_t> words;
    for (size_t w = 0; w < 10; w++)
    {
        size_t wordStartBit = preambleIdx + w * 30;
        if (wordStartBit + 30 > bits.size()) break;

        uint32_t word = 0;
        for (int b = 0; b < 30; b++)
        {
            bool bitVal = bits[wordStartBit + b];
            if (inverted) bitVal = !bitVal;
            word = (word << 1) | (bitVal ? 1 : 0);
        }
        words.push_back(word);
    }

    if (words.size() >= 10)
    {
        return decodeSubframe(words, ephem);
    }

    return false;
}
