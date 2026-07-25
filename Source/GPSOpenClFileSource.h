#ifndef INCLUDED_GPSOPENCL_FILESOURCE_H
#define INCLUDED_GPSOPENCL_FILESOURCE_H

#include "GPSOpenClSource.h"
#include <fstream>
#include <string>
#include <vector>

namespace GPSOpenCl
{
class FileSource : public Source
{
  public:
    FileSource();
    ~FileSource() override;

    bool initialize(const SourceInput &input) override;
    bool readBlock(ComplexFloatVector &outputSamples, SourceOutput &telemetry) override;

    bool loadAllSamples(const std::string &filePath, size_t samplesPerBlock);

  private:
    SourceInput m_inputConfig;
    ComplexFloatVector m_allSamples;
    size_t m_currentBlockIndex;
    size_t m_samplesPerBlock;
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_FILESOURCE_H
