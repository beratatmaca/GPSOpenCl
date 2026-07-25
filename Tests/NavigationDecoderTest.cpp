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
}
