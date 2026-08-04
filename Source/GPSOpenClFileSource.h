#ifndef INCLUDED_GPSOPENCL_FILESOURCE_H
#define INCLUDED_GPSOPENCL_FILESOURCE_H

/** @file GPSOpenClFileSource.h
 *  @brief File-based IQ sample source.
 */

#include "GPSOpenClSource.h"
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

    /** @brief Load all samples from file at once.
     *  @param filePath        File path.
     *  @param samplesPerBlock Samples per block.
     *  @return True if loaded. */
    bool loadAllSamples(const std::string &filePath, size_t samplesPerBlock);

  private:
    SourceInput m_inputConfig;          ///< Source configuration.
    ComplexFloatVector m_allSamples;    ///< All loaded samples.
    size_t m_currentBlockIndex;         ///< Current block index.
    size_t m_samplesPerBlock;           ///< Samples per block.
};
}

#endif
