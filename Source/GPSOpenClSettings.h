#ifndef INCLUDED_GPSOPENCL_SETTINGS_H
#define INCLUDED_GPSOPENCL_SETTINGS_H

/** @file GPSOpenClSettings.h
 *  @brief INI configuration file parser.
 */

#include <map>
#include <string>

#include "GPSOpenClCommon.h"
#include "GPSOpenClStructs.h"

namespace GPSOpenCl
{
/** @brief Configuration file parser and settings container. */
class Settings
{
  public:
    Settings();
    ~Settings();
    Settings(const Settings &) = delete;
    Settings &operator=(const Settings &) = delete;
    Settings(Settings &&) = delete;
    Settings &operator=(Settings &&) = delete;

    /** @brief Parse the INI configuration file. */
    void captureSettings();

    /** @brief Raw data source settings. */
    struct RawDataSettings
    {
        std::string dataSource;          ///< Data source file path.
        float samplingFrequency{};       ///< Sampling rate (Hz).
        int numberOfSamplesPerCode{};    ///< Samples per code period.
    };

    /** @brief Acquisition search parameters. */
    struct AcquisitionSettings
    {
        int acquisitionDopplerMinimum;        ///< Min Doppler search (Hz).
        int acquisitionDopplerMaximum;        ///< Max Doppler search (Hz).
        int acquisitionDopplerSearchRange;    ///< Doppler bin step (Hz).
    };

    /** @brief Full application configuration. */
    struct Configuration
    {
        RawDataSettings rawDataSettings;              ///< Raw data settings.
        AcquisitionSettings acquisitionSettings{};    ///< Acquisition settings.
        SourceInput sourceInput;                      ///< Source module input.
        AcquisitionInput acquisitionInput;            ///< Acquisition module input.
        TrackingInput trackingInput;                  ///< Tracking module input.
        NavDecoderInput navDecoderInput;              ///< Nav decoder module input.
        PvtSolverInput pvtSolverInput;                ///< PVT solver module input.
        AtmosphericInput atmosphericInput;            ///< Atmospheric module input.
        NmeaGeneratorInput nmeaGeneratorInput;        ///< NMEA generator module input.
    } configuration;

  private:
    /** @brief Parse one key=value line.
     *  @param line INI file line. */
    void fillMap(const std::string &line);

    /** @brief Update configuration struct from parsed map. */
    void updateConfigurationStruct();

    /** @brief Trim whitespace from string.
     *  @param str        Input string.
     *  @param whitespace Characters to trim.
     *  @return Trimmed string. */
    static std::string trim(const std::string &str, const std::string &whitespace);

    std::string m_confFileName;                               ///< Config file path.
    std::map<std::string, std::string> m_configurationMap;    ///< Parsed key-value pairs.
};
}

#endif
