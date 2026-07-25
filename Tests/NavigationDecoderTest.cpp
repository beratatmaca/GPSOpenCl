#include "GPSOpenClNavigationDecoder.h"

#include "gtest/gtest.h"

namespace GPSOpenClTest
{
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
}
