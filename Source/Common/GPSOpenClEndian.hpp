#ifndef INCLUDED_GPSOPENCL_ENDIAN_HPP
#define INCLUDED_GPSOPENCL_ENDIAN_HPP

/** @file GPSOpenClEndian.hpp
 *  @brief Runtime byte-swap for little-endian wire structs. No per-platform preprocessor branch:
 *   every host takes the same code path, and swapping is skipped at runtime when it would be a no-op.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace GPSOpenCl
{
/** @brief Detect host byte order at runtime.
 *  @return True if the host is little-endian. */
inline bool isHostLittleEndian()
{
    constexpr uint16_t PROBE = 0x0001;
    unsigned char firstByte = 0;
    std::memcpy(&firstByte, &PROBE, 1);
    return firstByte == 0x01;
}

/** @brief Reverse the byte order of an arithmetic value. Unconditional; callers gate on host
 *   endianness. Exposed separately so tests can verify the swap itself on any host.
 *  @param value Value to reverse.
 *  @return Byte-reversed value. */
template<typename T> T swapBytes(T value)
{
    static_assert(std::is_arithmetic_v<T>, "swapBytes requires an arithmetic wire field type");
    T swapped{};
    const auto *src = reinterpret_cast<const unsigned char *>(&value);
    auto *dst = reinterpret_cast<unsigned char *>(&swapped);
    for (size_t i = 0; i < sizeof(T); i++)
    {
        dst[i] = src[sizeof(T) - 1 - i];
    }
    return swapped;
}

/** @brief Convert one wire field from host to little-endian in place. A no-op on little-endian
 *   hosts and on single-byte fields.
 *  @param value Field to convert. */
template<typename T> void hostToLittleEndianInPlace(T &value)
{
    if constexpr (sizeof(T) > 1)
    {
        if (!isHostLittleEndian())
        {
            value = swapBytes(value);
        }
    }
}

/** @brief Convert every given wire field from host to little-endian in place.
 *  @param fields Wire struct fields, typically from a struct's wireFields() tie. */
template<typename... Fields> void swapFieldsToLittleEndian(Fields &...fields)
{
    (hostToLittleEndianInPlace(fields), ...);
}
}

#endif
