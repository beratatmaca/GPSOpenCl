#ifndef INCLUDED_GPSOPENCL_GPSSDRSIMSOURCE_H
#define INCLUDED_GPSOPENCL_GPSSDRSIMSOURCE_H

/** @file GPSOpenClGpsSdrSimSource.h
 *  @brief gps-sdr-sim FIFO sample source.
 */

#include "GPSOpenClSource.h"
#include <cstdint>
#include <string>
#include <vector>

namespace GPSOpenCl
{
/** @brief Sample source that reads IQ data from a gps-sdr-sim FIFO. */
class GpsSdrSimSource : public Source
{
  public:
    GpsSdrSimSource();
    ~GpsSdrSimSource() override;
    GpsSdrSimSource(const GpsSdrSimSource &) = delete;
    GpsSdrSimSource &operator=(const GpsSdrSimSource &) = delete;
    GpsSdrSimSource(GpsSdrSimSource &&) = delete;
    GpsSdrSimSource &operator=(GpsSdrSimSource &&) = delete;

    /** @brief Initialize from source config.
     *  @param input Source configuration.
     *  @return True if initialized. */
    bool initialize(const SourceInput &input) override;

    /** @brief Read one block of samples from FIFO.
     *  @param outputSamples Output sample buffer.
     *  @param telemetry     Output telemetry.
     *  @return True if block read. */
    bool readBlock(ComplexFloatVector &outputSamples, SourceOutput &telemetry) override;

    /** @brief Send a control command to the simulator.
     *  @param command Command string (e.g. START, STOP).
     *  @return True if sent. */
    bool sendControlCommand(const std::string &command);

  private:
    SourceInput m_inputConfig;           ///< Source configuration.
    std::string m_dataFifoPath;          ///< Data FIFO path.
    std::string m_ctrlFifoPath;          ///< Control FIFO path.
    int m_dataFd{-1};                    ///< Data FIFO file descriptor.
    int m_ctrlFd{-1};                    ///< Control FIFO file descriptor.
    uint32_t m_blockIndex{0};            ///< Current block index.
    uint32_t m_underrunCount{0};         ///< FIFO underrun count.
    uint32_t m_overrunCount{0};          ///< FIFO overrun count.
    size_t m_samplesPerBlock{4096};      ///< Samples per block, computed once in initialize().
    std::vector<int8_t> m_byteBuffer;    ///< Reused staging buffer for raw FIFO bytes.
};
}

#endif
