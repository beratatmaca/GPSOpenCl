#ifndef INCLUDED_GPSOPENCL_NAVIGATIONDECODER_H
#define INCLUDED_GPSOPENCL_NAVIGATIONDECODER_H

/** @file GPSOpenClNavigationDecoder.h
 *  @brief Navigation message decoder: preamble search, parity check, ephemeris parsing.
 */

#include "GPSOpenClCommon.h"

#include <cstdint>
#include <memory>
#include <vector>

#include "GPSOpenClSink.h"
#include "GPSOpenClStructs.h"

namespace GPSOpenCl
{
/** @brief Decoded GPS ephemeris and clock parameters. */
struct GpsEphemeris
{
    int svId;          ///< Satellite vehicle ID.
    int weekNumber;    ///< GPS week number.
    double tow;        ///< Time of week (s).
    int subframeId;    ///< Subframe ID (1-5).
    bool isValid;      ///< True if decode valid.

    double toc;        ///< Clock reference time (s).
    double af0;        ///< Clock bias (s).
    double af1;        ///< Clock drift (s/s).
    double af2;        ///< Clock drift rate (s/s^2).

    double toe;        ///< Ephemeris reference time (s).
    double sqrtA;      ///< Sqrt of semi-major axis (m^1/2).
    double e;          ///< Orbital eccentricity.
    double i0;         ///< Inclination at reference time (rad).
    double omega0;     ///< Longitude of ascending node (rad).
    double omega;      ///< Argument of perigee (rad).
    double M0;         ///< Mean anomaly at reference time (rad).
    double deltaN;     ///< Mean motion correction (rad/s).
    double omegaDot;   ///< Rate of right ascension (rad/s).
    double idot;       ///< Rate of inclination (rad/s).
    double Cuc, Cus, Crc, Crs, Cic, Cis; ///< Harmonic correction terms.
};

/** @brief GPS L1 C/A navigation message decoder. */
class NavigationDecoder
{
  public:
    NavigationDecoder();

    /** @brief Construct from decoder parameters.
     *  @param input Decoder settings. */
    NavigationDecoder(const NavDecoderInput &input);

    ~NavigationDecoder();

    /** @brief Find TLM preamble in a bit stream.
     *  @param bits          Navigation bit array.
     *  @param preambleIndex Index of preamble (output).
     *  @param inverted      True if preamble is inverted (output).
     *  @return True if preamble found. */
    static bool findPreamble(const std::vector<bool> &bits, size_t &preambleIndex, bool &inverted);

    /** @brief Verify IS-GPS-200 parity on a 30-bit word.
     *  @param word30bit 30-bit navigation word.
     *  @param prevD29   Previous word bit 29.
     *  @param prevD30   Previous word bit 30.
     *  @return True if parity valid. */
    static bool checkParity(uint32_t word30bit, bool prevD29, bool prevD30);

    /** @brief Extract signed bits from a packed word.
     *  @param val      Packed word.
     *  @param startBit Start bit position.
     *  @param numBits  Number of bits.
     *  @return Signed value. */
    static int32_t extractSignedBits(uint32_t val, int startBit, int numBits);

    /** @brief Extract unsigned bits from a packed word.
     *  @param val      Packed word.
     *  @param startBit Start bit position.
     *  @param numBits  Number of bits.
     *  @return Unsigned value. */
    static uint32_t extractUnsignedBits(uint32_t val, int startBit, int numBits);

    /** @brief Convert GpsEphemeris to NavDecoderOutput struct.
     *  @param ephem Source ephemeris.
     *  @return Output struct. */
    static NavDecoderOutput ephemerisToOutput(const GpsEphemeris &ephem);

    /** @brief Convert NavDecoderOutput to GpsEphemeris struct.
     *  @param out Source output struct.
     *  @return Ephemeris struct. */
    static GpsEphemeris outputToEphemeris(const NavDecoderOutput &out);

    /** @brief Decode one navigation subframe into ephemeris.
     *  @param words30bit Vector of 10 parity-checked 30-bit words.
     *  @param ephem      Output ephemeris.
     *  @return True if subframe decoded. */
    bool decodeSubframe(const std::vector<uint32_t> &words30bit, GpsEphemeris &ephem);

    /** @brief Decode one navigation subframe into output struct.
     *  @param words30bit Vector of 10 parity-checked 30-bit words.
     *  @param output     Output struct.
     *  @return True if subframe decoded. */
    bool decodeSubframe(const std::vector<uint32_t> &words30bit, NavDecoderOutput &output);

    /** @brief Convert Prompt correlator history to navigation bits.
     *  @param promptHistory Accumulated Prompt I samples.
     *  @return Demodulated bit array. */
    std::vector<bool> promptToBits(const ComplexFloatVector &promptHistory);

    /** @brief Process prompt signal and decode subframe into ephemeris.
     *  @param svId                Satellite vehicle ID.
     *  @param promptHistory       Accumulated Prompt I samples.
     *  @param bitOffset           Current bit offset (in/out).
     *  @param ephem               Output ephemeris.
     *  @param subframeStartSample Subframe start sample index (output).
     *  @return True if subframe decoded. */
    bool processPromptSignal(int svId, const ComplexFloatVector &promptHistory, size_t &bitOffset,
                             GpsEphemeris &ephem, size_t &subframeStartSample);

    /** @brief Process prompt signal and decode subframe into output struct.
     *  @param svId                Satellite vehicle ID.
     *  @param promptHistory       Accumulated Prompt I samples.
     *  @param bitOffset           Current bit offset (in/out).
     *  @param output              Output struct.
     *  @param subframeStartSample Subframe start sample index (output).
     *  @return True if subframe decoded. */
    bool processPromptSignal(int svId, const ComplexFloatVector &promptHistory, size_t &bitOffset,
                             NavDecoderOutput &output, size_t &subframeStartSample);

    /** @brief Set telemetry sink.
     *  @param sink Sink implementation. */
    void setSink(std::shared_ptr<Sink> sink) { m_sink = sink; }

    /** @brief Check if broadcast ionospheric parameters have been decoded.
     *  @return True if Subframe 4 Page 18 has been seen. */
    bool hasIonosphericParams() const { return m_hasIonoParams; }

    /** @brief Get decoded broadcast Klobuchar ionospheric parameters.
     *  @return Alpha/beta coefficients (zero if never decoded). */
    const AtmosphericInput &getIonosphericParams() const { return m_ionoParams; }

  private:
    /** @brief Decode Subframe 4 Page 18 ionospheric/UTC alpha/beta coefficients if present.
     *  @param words30bit Vector of 10 parity-checked 30-bit words. */
    void decodeIonosphericParams(const std::vector<uint32_t> &words30bit);

    NavDecoderInput m_inputConfig;            ///< Decoder parameters.
    std::shared_ptr<Sink> m_sink{nullptr};    ///< Telemetry sink.
    AtmosphericInput m_ionoParams{};          ///< Decoded broadcast Klobuchar coefficients.
    bool m_hasIonoParams{false};              ///< True once Subframe 4 Page 18 is decoded.
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_NAVIGATIONDECODER_H
