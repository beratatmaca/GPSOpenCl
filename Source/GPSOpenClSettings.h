#ifndef INCLUDED_GPSOPENCL_SETTINGS_H
#define INCLUDED_GPSOPENCL_SETTINGS_H

#include <map>
#include <string>

#include "GPSOpenClCommon.h"

namespace GPSOpenCl
{
class Settings
{
  public:
    Settings();
    ~Settings();

    void captureSettings();

    struct RawDataSettings
    {
        std::string dataSource;
        float samplingFrequency;
        int numberOfSamplesPerCode;
    };

    struct AcquisitionSettings
    {
        int acquisitionDopplerMinimum;
        int acquisitionDopplerMaximum;
        int acquisitionDopplerSearchRange;
    };

    struct Configuration
    {
        RawDataSettings rawDataSettings;
        AcquisitionSettings acquisitionSettings;
    } configuration;

  private:
    void fillMap(std::string line);
    void updateConfigurationStruct();
    std::string trim(const std::string &str, const std::string &whitespace);

    std::string m_confFileName;
    std::map<std::string, std::string> m_configurationMap;
};
} // namespace GPSOpenCl

#endif //! INCLUDED_GPSOPENCL_SETTINGS_H