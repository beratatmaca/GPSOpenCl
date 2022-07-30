#include "GPSOpenClSettings.h"

#include <fstream>
#include <iostream>

using namespace GPSOpenCl;

/**
 * @brief Construct a new Settings::Settings object
 *
 */
Settings::Settings()
{
    m_confFileName = "DefaultConf.ini";
}

/**
 * @brief Destroy the Settings::Settings object
 *
 */
Settings::~Settings()
{
}

/**
 * @brief
 *
 */
void Settings::captureSettings()
{
    std::ifstream confFile;
    std::string line;

    confFile.open(m_confFileName);

    if (!confFile)
    {
        std::cerr << "Error in opening the configuration file" << std::endl;
    }
    else
    {
        while (getline(confFile, line))
        {
            fillMap(line);
        }
    }

    confFile.close();

    updateConfigurationStruct();
}

/**
 * @brief
 *
 * @param line
 */
void Settings::fillMap(std::string line)
{
    std::string::size_type keyPos = 0;
    std::string::size_type keyEnd;
    std::string::size_type valPos;
    std::string::size_type valEnd;
    std::string key;
    std::string value;

    while ((keyEnd = line.find('=', keyPos)) != std::string::npos)
    {
        if ((valPos = line.find_first_not_of("= ", keyEnd)) == std::string::npos)
        {
            break;
        }
        else
        {
            valEnd = line.length();

            key = line.substr(keyPos, keyEnd - keyPos);
            value = line.substr(valPos, valEnd - valPos);

            key = trim(key, " ");
            value = trim(value, " ");

            m_configurationMap.emplace(key, value);
        }
        keyPos = valEnd;
    }
}

/**
 * @brief
 *
 */
void Settings::updateConfigurationStruct()
{
    if (m_configurationMap["DataSource"] != "")
    {
        configuration.rawDataSettings.dataSource = m_configurationMap["DataSource"];
    }

    if (m_configurationMap["SamplingFrequency"] != "")
    {
        configuration.rawDataSettings.samplingFrequency = std::stof(m_configurationMap["SamplingFrequency"]);
        configuration.rawDataSettings.numberOfSamplesPerCode = static_cast<int>(std::round(
            configuration.rawDataSettings.samplingFrequency / (GPS_CA_CODE_FREQUENCY_HZ / GPS_CA_CODE_LENGTH)));
    }

    if (m_configurationMap["AcquisitionMinimumDoppler"] != "")
    {
        configuration.acquisitionSettings.acquisitionDopplerMinimum =
            std::stoi(m_configurationMap["AcquisitionMinimumDoppler"]);
    }

    if (m_configurationMap["AcquisitionMaximumDoppler"] != "")
    {
        configuration.acquisitionSettings.acquisitionDopplerMaximum =
            std::stoi(m_configurationMap["AcquisitionMaximumDoppler"]);
    }

    if (m_configurationMap["AcquisitionDopplerSearchRange"] != "")
    {
        configuration.acquisitionSettings.acquisitionDopplerSearchRange =
            std::stoi(m_configurationMap["AcquisitionDopplerSearchRange"]);
    }
}

/**
 * @brief
 *
 * @param str
 * @param whitespace
 * @return std::string
 */
std::string Settings::trim(const std::string &str, const std::string &whitespace = " \t")
{
    const auto strBegin = str.find_first_not_of(whitespace);
    if (strBegin == std::string::npos)
    {
        return "";
    }

    const auto strEnd = str.find_last_not_of(whitespace);

    const auto strRange = strEnd - strBegin + 1;

    return str.substr(strBegin, strRange);
}