#ifndef INCLUDED_GPSOPENCL_FILESOURCE_HPP
#define INCLUDED_GPSOPENCL_FILESOURCE_HPP

/** @file GPSOpenClFileSource.hpp
 *  @brief File-based IQ sample source.
 */

#include "Input/GPSOpenClSource.hpp"
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace GPSOpenCl
{
/** @brief Sample source that reads IQ data from a binary file. */
class FileSource : public Source
{
  public:
    FileSource();
    ~FileSource() override;
    FileSource(const FileSource &) = delete;
    FileSource &operator=(const FileSource &) = delete;
    FileSource(FileSource &&) = delete;
    FileSource &operator=(FileSource &&) = delete;

    /** @brief Initialize from source config.
     *  @param input Source configuration.
     *  @return True if initialized. */
    bool initialize(const SourceInput &input) override;

    /** @brief Read one block of samples.
     *  @param outputSamples Output sample buffer.
     *  @param telemetry     Output telemetry.
     *  @return True if block read. */
    bool readBlock(ComplexFloatVector &outputSamples, SourceOutput &telemetry) override;

    /** @brief Load all samples from a text file at once. Binary captures stream block by block in
     *   readBlock(). Resident memory then stays one block.
     *  @param filePath        File path.
     *  @param samplesPerBlock Samples per block.
     *  @return True if loaded. */
    bool loadAllSamples(const std::string &filePath, size_t samplesPerBlock);

  private:
    SourceInput m_inputConfig;           ///< Source configuration.
    ComplexFloatVector m_allSamples;     ///< All loaded samples (text-file path only).
    size_t m_currentBlockIndex{0};       ///< Current block index.
    size_t m_samplesPerBlock{4096};      ///< Samples per block.
    std::ifstream m_binFile;             ///< Open stream for a binary capture.
    bool m_streaming{false};             ///< True when reading a .bin capture block by block.
    size_t m_totalSamples{0};            ///< Total samples in the streamed capture.
    std::vector<int8_t> m_byteBuffer;    ///< Reused staging buffer for raw file bytes.
};
}

#endif
