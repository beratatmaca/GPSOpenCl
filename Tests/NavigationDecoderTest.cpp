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
} // namespace GPSOpenClTest
