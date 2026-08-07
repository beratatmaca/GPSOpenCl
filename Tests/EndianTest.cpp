#include "gtest/gtest.h"

#include "Common/GPSOpenClEndian.hpp"
#include "Common/GPSOpenClStructs.hpp"

#include <cstring>

using namespace GPSOpenCl;

// x86_64 and ARM64 (this project's only deployment targets) are always little-endian.
TEST(EndianTest, HostIsLittleEndian)
{
    EXPECT_TRUE(isHostLittleEndian());
}

TEST(EndianTest, SwapBytesReversesByteOrder)
{
    const uint32_t value = 0x01'02'03'04;
    const uint32_t swapped = swapBytes(value);

    EXPECT_EQ(swapped, 0x04'03'02'01u);

    // The swapped value's little-endian memory bytes are the original value's bytes, reversed.
    unsigned char expected[4] = {0x01, 0x02, 0x03, 0x04};
    unsigned char actual[4];
    std::memcpy(actual, &swapped, sizeof(actual));
    EXPECT_EQ(std::memcmp(expected, actual, sizeof(actual)), 0);
}

TEST(EndianTest, SwapBytesTwiceRoundTrips)
{
    const double value = 123456.789;
    EXPECT_EQ(swapBytes(swapBytes(value)), value);

    const uint32_t intValue = 0xDE'AD'BE'EF;
    EXPECT_EQ(swapBytes(swapBytes(intValue)), intValue);
}

TEST(EndianTest, HostToLittleEndianInPlaceIsNoOpOnLittleEndianHost)
{
    uint32_t intValue = 0x11'22'33'44;
    double doubleValue = 42.5;

    hostToLittleEndianInPlace(intValue);
    hostToLittleEndianInPlace(doubleValue);

    EXPECT_EQ(intValue, 0x11'22'33'44u);
    EXPECT_EQ(doubleValue, 42.5);
}

TEST(EndianTest, SwapFieldsToLittleEndianAppliesToEveryArgument)
{
    uint32_t a = 1;
    double b = 2.0;
    int32_t c = -3;

    swapFieldsToLittleEndian(a, b, c);

    // No-op on this (little-endian) host; verifies the fold expression visits every field
    // without a compile error or crash, for a mix of field widths and signedness.
    EXPECT_EQ(a, 1u);
    EXPECT_EQ(b, 2.0);
    EXPECT_EQ(c, -3);
}

TEST(EndianTest, WireFieldsTiesReferenceTheOriginalStruct)
{
    SourceOutput out{};
    out.structVersion = 7;
    out.blockIndex = 8;
    out.timestampSec = 9.0;
    out.fifoUnderrunCount = 10;
    out.fifoOverrunCount = 11;

    auto fields = out.wireFields();
    std::get<0>(fields) = 70;
    std::get<3>(fields) = 100;

    EXPECT_EQ(out.structVersion, 70u);
    EXPECT_EQ(out.fifoUnderrunCount, 100u);
    EXPECT_EQ(out.blockIndex, 8u);
}
