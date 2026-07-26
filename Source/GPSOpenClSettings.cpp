#include "GPSOpenClSettings.h"

#include <fstream>
#include <iostream>
#include <vector>

using namespace GPSOpenCl;





Settings::Settings()
{
    m_confFileName = "DefaultConf.ini";
    configuration.rawDataSettings.dataSource = "capture.dat";
    configuration.rawDataSettings.samplingFrequency = 4096000.0f;
    configuration.rawDataSettings.numberOfSamplesPerCode = 4096;
    configuration.acquisitionSettings.acquisitionDopplerMinimum = -4000;
    configuration.acquisitionSettings.acquisitionDopplerMaximum = 4000;
    configuration.acquisitionSettings.acquisitionDopplerSearchRange = 500;
}





Settings::~Settings()
{
}





void Settings::captureSettings()
{
    std::ifstream confFile;
    std::string line;

    std::vector<std::string> candidatePaths = {
        m_confFileName,
        "build/Source/" + m_confFileName,
        "Tests/Scripts/ConfigurationFile/" + m_confFileName,
        "../Tests/Scripts/ConfigurationFile/" + m_confFileName,
        "../../Tests/Scripts/ConfigurationFile/" + m_confFileName
    };

    bool opened = false;
    for (const auto &path : candidatePaths)
    {
        confFile.open(path);
        if (confFile.is_open())
        {
            opened = true;
            break;
        }
    }

    if (!opened)
    {
        std::cerr << "Error in opening the configuration file. Using default parameters." << std::endl;
    }
    else
    {
        while (getline(confFile, line))
        {
            fillMap(line);
        }
        confFile.close();
    }

    updateConfigurationStruct();
}






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





void Settings::updateConfigurationStruct()
{
    if (m_configurationMap["DataSource"] != "")
    {
        configuration.rawDataSettings.dataSource = m_configurationMap["DataSource"];
        snprintf(configuration.sourceInput.fifoPath, sizeof(configuration.sourceInput.fifoPath), "%s",
                 m_configurationMap["DataSource"].c_str());
    }

    if (m_configurationMap["SamplingFrequency"] != "")
    {
        try
        {
            configuration.rawDataSettings.samplingFrequency = std::stof(m_configurationMap["SamplingFrequency"]);
            configuration.rawDataSettings.numberOfSamplesPerCode = static_cast<int>(std::round(
                configuration.rawDataSettings.samplingFrequency / (GPS_CA_CODE_FREQUENCY_HZ / GPS_CA_CODE_LENGTH)));
            configuration.sourceInput.samplingRate = configuration.rawDataSettings.samplingFrequency;
            configuration.acquisitionInput.samplingFrequency = configuration.rawDataSettings.samplingFrequency;
            configuration.acquisitionInput.numberOfSamplesPerCode = configuration.rawDataSettings.numberOfSamplesPerCode;
            configuration.trackingInput.samplingFrequency = configuration.rawDataSettings.samplingFrequency;
            configuration.trackingInput.numberOfSamplesPerCode = configuration.rawDataSettings.numberOfSamplesPerCode;

            int samplesPerCode = configuration.rawDataSettings.numberOfSamplesPerCode;
            bool isPowerOfTwo = samplesPerCode > 0 && (samplesPerCode & (samplesPerCode - 1)) == 0;
            if (!isPowerOfTwo)
            {
                std::cerr << "Warning: SamplingFrequency=" << configuration.rawDataSettings.samplingFrequency
                          << " yields " << samplesPerCode << " samples per code period, which is not a power of "
                          << "two. The FFT-based acquisition/lookup-table path requires a power-of-two length and "
                          << "will fail for this configuration." << std::endl;
            }
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid SamplingFrequency value in config, using default." << std::endl;
        }
    }

    if (m_configurationMap["AcquisitionMinimumDoppler"] != "")
    {
        try
        {
            configuration.acquisitionSettings.acquisitionDopplerMinimum =
                std::stoi(m_configurationMap["AcquisitionMinimumDoppler"]);
            configuration.acquisitionInput.acquisitionDopplerMinimum =
                configuration.acquisitionSettings.acquisitionDopplerMinimum;
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid AcquisitionMinimumDoppler value in config, using default." << std::endl;
        }
    }

    if (m_configurationMap["AcquisitionMaximumDoppler"] != "")
    {
        try
        {
            configuration.acquisitionSettings.acquisitionDopplerMaximum =
                std::stoi(m_configurationMap["AcquisitionMaximumDoppler"]);
            configuration.acquisitionInput.acquisitionDopplerMaximum =
                configuration.acquisitionSettings.acquisitionDopplerMaximum;
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid AcquisitionMaximumDoppler value in config, using default." << std::endl;
        }
    }

    if (m_configurationMap["AcquisitionDopplerSearchRange"] != "")
    {
        try
        {
            configuration.acquisitionSettings.acquisitionDopplerSearchRange =
                std::stoi(m_configurationMap["AcquisitionDopplerSearchRange"]);
            configuration.acquisitionInput.acquisitionDopplerSearchRange =
                configuration.acquisitionSettings.acquisitionDopplerSearchRange;
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid AcquisitionDopplerSearchRange value in config, using default." << std::endl;
        }
    }

    if (m_configurationMap["PllBandwidthHz"] != "")
    {
        try
        {
            configuration.trackingInput.pllBandwidthHz = std::stod(m_configurationMap["PllBandwidthHz"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid PllBandwidthHz value in config, using default." << std::endl;
        }
    }

    if (m_configurationMap["DllBandwidthHz"] != "")
    {
        try
        {
            configuration.trackingInput.dllBandwidthHz = std::stod(m_configurationMap["DllBandwidthHz"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid DllBandwidthHz value in config, using default." << std::endl;
        }
    }

    if (m_configurationMap["FllBandwidthHz"] != "")
    {
        try
        {
            configuration.trackingInput.fllBandwidthHz = std::stod(m_configurationMap["FllBandwidthHz"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid FllBandwidthHz value in config, using default." << std::endl;
        }
    }

    if (m_configurationMap["RateAidBandwidthHz"] != "")
    {
        try
        {
            configuration.trackingInput.rateAidBandwidthHz = std::stod(m_configurationMap["RateAidBandwidthHz"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid RateAidBandwidthHz value in config, using default." << std::endl;
        }
    }

    if (m_configurationMap["FllPullInBlocks"] != "")
    {
        try
        {
            configuration.trackingInput.fllPullInBlocks = std::stoi(m_configurationMap["FllPullInBlocks"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid FllPullInBlocks value in config, using default." << std::endl;
        }
    }

    if (m_configurationMap["FixOutputIntervalBlocks"] != "")
    {
        try
        {
            configuration.pvtSolverInput.fixOutputIntervalBlocks =
                std::stoi(m_configurationMap["FixOutputIntervalBlocks"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid FixOutputIntervalBlocks value in config, using default." << std::endl;
        }
    }
}








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
