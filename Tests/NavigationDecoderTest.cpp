#include "GPSOpenClNavigationDecoder.h"

#include "gtest/gtest.h"
#include <cmath>

namespace GPSOpenClTest
{
static uint32_t packBits(uint32_t value, int startBitFromMSB, int numBits)
{
    uint32_t mask = (1u << numBits) - 1;
    int shift = 30 - (startBitFromMSB + numBits - 1);
    return (value & mask) << shift;
}
TEST(NavigationDecoderTest, PreambleSearchNormal)
{
    std::vector<bool> bits;
    for (int i = 0; i < 50; i++) bits.push_back(false);

    bits.push_back(true);
    bits.push_back(false);
    bits.push_back(false);
    bits.push_back(false);
    bits.push_back(true);
    bits.push_back(false);
    bits.push_back(true);
    bits.push_back(true);

    while (bits.size() < 350) bits.push_back(false);

    size_t preambleIdx = 0;
    bool inverted = false;
    bool found = GPSOpenCl::NavigationDecoder::findPreamble(bits, preambleIdx, inverted);

    EXPECT_TRUE(found);
    EXPECT_EQ(preambleIdx, 50u);
    EXPECT_FALSE(inverted);
}

TEST(NavigationDecoderTest, PreambleSearchInverted)
{
    std::vector<bool> bits;
    for (int i = 0; i < 20; i++) bits.push_back(false);

    bits.push_back(false);
    bits.push_back(true);
    bits.push_back(true);
    bits.push_back(true);
    bits.push_back(false);
    bits.push_back(true);
    bits.push_back(false);
    bits.push_back(false);

    while (bits.size() < 350) bits.push_back(false);

    size_t preambleIdx = 0;
    bool inverted = false;
    bool found = GPSOpenCl::NavigationDecoder::findPreamble(bits, preambleIdx, inverted);

    EXPECT_TRUE(found);
    EXPECT_EQ(preambleIdx, 20u);
    EXPECT_TRUE(inverted);
}

TEST(NavigationDecoderTest, BitExtraction)
{
    uint32_t word = 0x22C00000;
    uint32_t preamble = GPSOpenCl::NavigationDecoder::extractUnsignedBits(word, 1, 8);
    EXPECT_EQ(preamble, 0x8Bu);
}

TEST(NavigationDecoderTest, SubframeSearchMaskFiltersDisabledSubframe)
{
    std::vector<uint32_t> words(10, 0);
    words[1] = 0x300;

    GPSOpenCl::GpsEphemeris ephemAllEnabled{};
    GPSOpenCl::NavigationDecoder allEnabled;
    EXPECT_TRUE(allEnabled.decodeSubframe(words, ephemAllEnabled));
    EXPECT_EQ(ephemAllEnabled.subframeId, 3);

    GPSOpenCl::NavDecoderInput maskWithoutSubframe3{};
    maskWithoutSubframe3.structVersion = GPSOpenCl::STRUCT_VERSION_1;
    maskWithoutSubframe3.subframeSearchMask = 0x1B;

    GPSOpenCl::GpsEphemeris ephemMasked{};
    GPSOpenCl::NavigationDecoder subframe3Disabled(maskWithoutSubframe3);
    EXPECT_FALSE(subframe3Disabled.decodeSubframe(words, ephemMasked));
    EXPECT_FALSE(ephemMasked.isValid);
}

TEST(NavigationDecoderTest, DecodesSubframe4Page18IonosphericParams)
{
    int32_t alpha0Raw = 10, alpha1Raw = -5, alpha2Raw = 3, alpha3Raw = -2;
    int32_t beta0Raw = 20, beta1Raw = -10, beta2Raw = 5, beta3Raw = -3;

    std::vector<uint32_t> words(10, 0);
    words[1] = 4u << 8;
    words[2] = packBits(1, 1, 2) | packBits(56, 3, 6) |
               packBits(static_cast<uint32_t>(alpha0Raw), 9, 8) | packBits(static_cast<uint32_t>(alpha1Raw), 17, 8);
    words[3] = packBits(static_cast<uint32_t>(alpha2Raw), 1, 8) | packBits(static_cast<uint32_t>(alpha3Raw), 9, 8) |
               packBits(static_cast<uint32_t>(beta0Raw), 17, 8);
    words[4] = packBits(static_cast<uint32_t>(beta1Raw), 1, 8) | packBits(static_cast<uint32_t>(beta2Raw), 9, 8) |
               packBits(static_cast<uint32_t>(beta3Raw), 17, 8);

    GPSOpenCl::NavigationDecoder decoder;
    GPSOpenCl::GpsEphemeris ephem{};
    EXPECT_FALSE(decoder.decodeSubframe(words, ephem));
    ASSERT_TRUE(decoder.hasIonosphericParams());

    const GPSOpenCl::AtmosphericInput &params = decoder.getIonosphericParams();
    EXPECT_NEAR(params.alpha0, alpha0Raw * std::pow(2.0, -30), 1e-15);
    EXPECT_NEAR(params.alpha1, alpha1Raw * std::pow(2.0, -27), 1e-15);
    EXPECT_NEAR(params.alpha2, alpha2Raw * std::pow(2.0, -24), 1e-15);
    EXPECT_NEAR(params.alpha3, alpha3Raw * std::pow(2.0, -24), 1e-15);
    EXPECT_NEAR(params.beta0, beta0Raw * std::pow(2.0, 11), 1e-6);
    EXPECT_NEAR(params.beta1, beta1Raw * std::pow(2.0, 14), 1e-6);
    EXPECT_NEAR(params.beta2, beta2Raw * std::pow(2.0, 16), 1e-6);
    EXPECT_NEAR(params.beta3, beta3Raw * std::pow(2.0, 16), 1e-6);
}

TEST(NavigationDecoderTest, IgnoresSubframe4PagesOtherThanPage18)
{
    std::vector<uint32_t> words(10, 0);
    words[1] = 4u << 8;
    words[2] = packBits(1, 1, 2) | packBits(63, 3, 6);

    GPSOpenCl::NavigationDecoder decoder;
    GPSOpenCl::GpsEphemeris ephem{};
    EXPECT_FALSE(decoder.decodeSubframe(words, ephem));
    EXPECT_FALSE(decoder.hasIonosphericParams());
}

static uint32_t buildParityWord(uint32_t dataBits30, bool prevD29, bool prevD30)
{
    bool d[25];
    for (int i = 1; i <= 24; i++)
    {
        d[i] = ((dataBits30 >> (30 - i)) & 1) != 0;
    }

    bool D25 = prevD29 ^ d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[17] ^ d[18] ^ d[20] ^ d[23];
    bool D26 = prevD30 ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[18] ^ d[19] ^ d[21] ^ d[24];
    bool D27 = prevD29 ^ d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[19] ^ d[20] ^ d[22];
    bool D28 = prevD30 ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[20] ^ d[21] ^ d[23];
    bool D29 = prevD30 ^ d[1] ^ d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[9] ^ d[10] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[21] ^ d[22] ^ d[24];
    bool D30 = prevD29 ^ d[3] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[10] ^ d[11] ^ d[13] ^ d[15] ^ d[19] ^ d[22] ^ d[23] ^ d[24];

    uint32_t parity = (D25 ? 1u : 0u) << 5 | (D26 ? 1u : 0u) << 4 | (D27 ? 1u : 0u) << 3 | (D28 ? 1u : 0u) << 2 |
                      (D29 ? 1u : 0u) << 1 | (D30 ? 1u : 0u);
    return dataBits30 | parity;
}

static std::vector<uint32_t> buildValidSubframe(const std::vector<uint32_t> &semanticWords30)
{
    std::vector<uint32_t> raw(10, 0);
    bool prevD29 = false;
    bool prevD30 = false;
    for (int w = 0; w < 10; w++)
    {
        uint32_t transmitted = prevD30 ? (semanticWords30[static_cast<size_t>(w)] ^ 0x3FFFFFC0u)
                                        : semanticWords30[static_cast<size_t>(w)];
        raw[static_cast<size_t>(w)] = buildParityWord(transmitted, prevD29, prevD30);
        prevD29 = ((raw[static_cast<size_t>(w)] >> 1) & 1u) != 0;
        prevD30 = (raw[static_cast<size_t>(w)] & 1u) != 0;
    }
    return raw;
}

static std::vector<bool> wordsToBits(const std::vector<uint32_t> &words30bit)
{
    std::vector<bool> bits;
    bits.reserve(words30bit.size() * 30);
    for (uint32_t raw : words30bit)
    {
        for (int b = 29; b >= 0; b--)
        {
            bits.push_back(((raw >> b) & 1u) != 0);
        }
    }
    return bits;
}

static GPSOpenCl::ComplexFloatVector bitsToPromptSamples(const std::vector<bool> &bits, int phase)
{
    GPSOpenCl::ComplexFloatVector samples;
    samples.reserve(static_cast<size_t>(phase) + bits.size() * 20);
    for (int i = 0; i < phase; i++)
    {
        samples.push_back(std::complex<float>(0.0f, 0.0f));
    }
    for (bool bit : bits)
    {
        float val = bit ? 1.0f : -1.0f;
        for (int k = 0; k < 20; k++)
        {
            samples.push_back(std::complex<float>(val, 0.0f));
        }
    }
    return samples;
}

TEST(NavigationDecoderTest, BitSyncFindsSubframeWhenNaiveZeroPhaseCannot)
{
    // Fully alternating data bits: any two adjacent bits always disagree, so a fixed
    // sample-phase-0 assumption against a signal truly offset by 10 samples (an exact
    // half-window split) forces every demodulated bit to a tied, unrecoverable zero-sum.
    const uint32_t alternatingFill = 0x555555u;

    std::vector<uint32_t> semanticWords(10, 0);
    semanticWords[0] = packBits(0x8B, 1, 8) | packBits(alternatingFill, 9, 16);
    semanticWords[1] = packBits(alternatingFill, 1, 19) | packBits(1, 20, 3) | packBits(alternatingFill, 23, 2);
    for (int w = 2; w < 10; w++)
    {
        semanticWords[static_cast<size_t>(w)] = packBits(alternatingFill, 1, 24);
    }

    std::vector<uint32_t> subframe = buildValidSubframe(semanticWords);
    std::vector<bool> bits = wordsToBits(subframe);

    const int truePhase = 10;
    GPSOpenCl::ComplexFloatVector promptHistory = bitsToPromptSamples(bits, truePhase);

    GPSOpenCl::NavigationDecoder naiveDecoder;
    std::vector<bool> naiveBits = naiveDecoder.promptToBits(promptHistory);
    size_t naivePreambleIdx = 0;
    bool naiveInverted = false;
    bool naiveFoundPreamble = GPSOpenCl::NavigationDecoder::findPreamble(naiveBits, naivePreambleIdx, naiveInverted);
    EXPECT_FALSE(naiveFoundPreamble) << "a fixed sample-phase-0 assumption should not survive a true 10-sample offset";

    GPSOpenCl::NavigationDecoder decoder;
    GPSOpenCl::GpsEphemeris ephem{};
    int bitSyncPhase = -1;
    std::vector<size_t> searchPositions;
    size_t bitOffset = 0;
    size_t subframeStartSample = 0;

    bool decoded = false;
    for (int attempt = 0; attempt < 20 && !decoded; attempt++)
    {
        decoded = decoder.processPromptSignal(1, promptHistory, bitSyncPhase, searchPositions, bitOffset, ephem,
                                              subframeStartSample);
    }

    EXPECT_TRUE(decoded);
    EXPECT_TRUE(ephem.isValid);
    EXPECT_EQ(ephem.subframeId, 1);
    EXPECT_GE(bitSyncPhase, 0);
    EXPECT_LT(bitSyncPhase, 20);
}

TEST(NavigationDecoderTest, BitSyncNeverLocksWithFewerThanOneSubframeOfSamples)
{
    std::vector<uint32_t> semanticWords(10, 0);
    semanticWords[0] = packBits(0x8B, 1, 8);
    semanticWords[1] = packBits(1, 20, 3);

    std::vector<uint32_t> subframe = buildValidSubframe(semanticWords);
    std::vector<bool> bits = wordsToBits(subframe);
    bits.resize(bits.size() - 1);

    GPSOpenCl::ComplexFloatVector promptHistory = bitsToPromptSamples(bits, 5);

    GPSOpenCl::NavigationDecoder decoder;
    GPSOpenCl::GpsEphemeris ephem{};
    int bitSyncPhase = -1;
    std::vector<size_t> searchPositions;
    size_t bitOffset = 0;
    size_t subframeStartSample = 0;

    bool decoded = decoder.processPromptSignal(1, promptHistory, bitSyncPhase, searchPositions, bitOffset, ephem,
                                               subframeStartSample);

    EXPECT_FALSE(decoded);
    EXPECT_EQ(bitSyncPhase, -1);
}
}
