#include "GPSOpenClNavigationDecoder.h"

#include "gtest/gtest.h"

namespace GPSOpenClTest
{
TEST(NavigationDecoderTest, PreambleSearchNormal)
{
    std::vector<bool> bits;
    // Fill 50 dummy bits before preamble
    for (int i = 0; i < 50; i++) bits.push_back(false);

    // Add Preamble 0x8B: 10001011
    bits.push_back(true);  // 1
    bits.push_back(false); // 0
    bits.push_back(false); // 0
    bits.push_back(false); // 0
    bits.push_back(true);  // 1
    bits.push_back(false); // 0
    bits.push_back(true);  // 1
    bits.push_back(true);  // 1

    // Fill remaining bits to reach >= 300 bits
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

    // Add Inverted Preamble 0x74: 01110100
    bits.push_back(false); // 0
    bits.push_back(true);  // 1
    bits.push_back(true);  // 1
    bits.push_back(true);  // 1
    bits.push_back(false); // 0
    bits.push_back(true);  // 1
    bits.push_back(false); // 0
    bits.push_back(false); // 0

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
    // 30-bit word: 0b100010110000000000000000000000 (0x22C00000)
    uint32_t word = 0x22C00000;
    uint32_t preamble = GPSOpenCl::NavigationDecoder::extractUnsignedBits(word, 1, 8);
    EXPECT_EQ(preamble, 0x8Bu);
}

TEST(NavigationDecoderTest, SubframeSearchMaskFiltersDisabledSubframe)
{
    std::vector<uint32_t> words(10, 0);
    words[1] = 0x300; // HOW word: subframeId = 3 (extractUnsignedBits(howWord, 20, 3) reads bits 8-10)

    GPSOpenCl::GpsEphemeris ephemAllEnabled{};
    GPSOpenCl::NavigationDecoder allEnabled; // default mask 0x1F: all subframes searched
    EXPECT_TRUE(allEnabled.decodeSubframe(words, ephemAllEnabled));
    EXPECT_EQ(ephemAllEnabled.subframeId, 3);

    GPSOpenCl::NavDecoderInput maskWithoutSubframe3{};
    maskWithoutSubframe3.structVersion = GPSOpenCl::STRUCT_VERSION_1;
    maskWithoutSubframe3.subframeSearchMask = 0x1B; // 0b11011: excludes bit index 2 (subframe 3)

    GPSOpenCl::GpsEphemeris ephemMasked{};
    GPSOpenCl::NavigationDecoder subframe3Disabled(maskWithoutSubframe3);
    EXPECT_FALSE(subframe3Disabled.decodeSubframe(words, ephemMasked));
    EXPECT_FALSE(ephemMasked.isValid);
}
} // namespace GPSOpenClTest
