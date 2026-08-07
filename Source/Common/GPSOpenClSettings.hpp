#ifndef INCLUDED_GPSOPENCL_SETTINGS_HPP
#define INCLUDED_GPSOPENCL_SETTINGS_HPP

/** @file GPSOpenClSettings.hpp
 *  @brief INI configuration file parser.
 */

#include <map>
#include <string>

#include "Common/GPSOpenClCommon.hpp"
#include "Common/GPSOpenClStructs.hpp"

namespace GPSOpenCl
{
/** @brief Configuration file parser and settings container. */
class Settings
{
  public:
    /** @brief Construct with built-in defaults, before captureSettings() loads the INI file. */
    Settings();

    ~Settings();
    Settings(const Settings &) = delete;
    Settings &operator=(const Settings &) = delete;
    Settings(Settings &&) = delete;
    Settings &operator=(Settings &&) = delete;

    /** @brief Parse the INI configuration file. */
    void captureSettings();

    /** @brief Full application configuration. */
    struct Configuration
    {
        SourceInput sourceInput;                  ///< Source module input.
        AcquisitionInput acquisitionInput;        ///< Acquisition module input.
        TrackingInput trackingInput;              ///< Tracking module input.
        NavDecoderInput navDecoderInput;          ///< Nav decoder module input.
        PvtSolverInput pvtSolverInput;            ///< PVT solver module input.
        AtmosphericInput atmosphericInput;        ///< Atmospheric module input.
        NmeaGeneratorInput nmeaGeneratorInput;    ///< NMEA generator module input.
        ProfilerInput profilerInput;              ///< Profiler module input.
    };

    Configuration configuration;    ///< Live configuration, populated by captureSettings().

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
