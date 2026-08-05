#include "GPSOpenClSettings.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace GPSOpenCl;

Settings::Settings() : m_confFileName("DefaultConf.ini")
{
    snprintf(configuration.sourceInput.fifoPath, sizeof(configuration.sourceInput.fifoPath), "capture.dat");
}

Settings::~Settings() = default;

void Settings::captureSettings()
{
    std::ifstream confFile;
    std::string line;

    const std::vector<std::string> candidatePaths = {m_confFileName,
                                                     "build/Source/" + m_confFileName,
                                                     "Tests/Scripts/ConfigurationFile/" + m_confFileName,
                                                     "../Tests/Scripts/ConfigurationFile/" + m_confFileName,
                                                     "../../Tests/Scripts/ConfigurationFile/" + m_confFileName};

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
        std::cerr << "Error in opening the configuration file. Using default parameters." << '\n';
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

void Settings::fillMap(const std::string &line)
{
    const std::string trimmedLine = trim(line, " \t");
    if (trimmedLine.empty() || trimmedLine[0] == '#' || trimmedLine[0] == ';' || trimmedLine[0] == '[')
    {
        return;
    }

    std::string::size_type keyPos = 0;
    std::string::size_type keyEnd = 0;
    std::string::size_type valPos = 0;
    std::string::size_type valEnd = 0;
    std::string key;
    std::string value;

    while ((keyEnd = line.find('=', keyPos)) != std::string::npos)
    {
        valPos = line.find_first_not_of("= ", keyEnd);
        if (valPos == std::string::npos)
        {
            break;
        }

        valEnd = line.length();

        key = line.substr(keyPos, keyEnd - keyPos);
        value = line.substr(valPos, valEnd - valPos);

        key = trim(key, " ");
        value = trim(value, " ");

        m_configurationMap.emplace(key, value);

        keyPos = valEnd;
    }
}

void Settings::updateConfigurationStruct()
{
    if (!m_configurationMap["DataSource"].empty())
    {
        snprintf(configuration.sourceInput.fifoPath,
                 sizeof(configuration.sourceInput.fifoPath),
                 "%s",
                 m_configurationMap["DataSource"].c_str());
    }

    if (!m_configurationMap["SamplingFrequency"].empty())
    {
        try
        {
            const float samplingFrequencyHz = std::stof(m_configurationMap["SamplingFrequency"]);
            const double samplesPerCodeReal =
                std::round(static_cast<double>(samplingFrequencyHz) / (GPS_CA_CODE_FREQUENCY_HZ / GPS_CA_CODE_LENGTH));
            if (!std::isfinite(samplingFrequencyHz) || samplingFrequencyHz <= 0.0f || samplesPerCodeReal < 1.0 ||
                samplesPerCodeReal > 16777216.0)
            {
                throw std::invalid_argument("SamplingFrequency out of range");
            }

            configuration.sourceInput.samplingRateHz = samplingFrequencyHz;
            configuration.acquisitionInput.samplingFrequencyHz = samplingFrequencyHz;
            configuration.acquisitionInput.numberOfSamplesPerCode = static_cast<int>(samplesPerCodeReal);
            configuration.trackingInput.samplingFrequencyHz = samplingFrequencyHz;
            configuration.trackingInput.numberOfSamplesPerCode = static_cast<int>(samplesPerCodeReal);

            const int samplesPerCode = configuration.acquisitionInput.numberOfSamplesPerCode;
            const bool isPowerOfTwo = samplesPerCode > 0 && (samplesPerCode & (samplesPerCode - 1)) == 0;
            if (!isPowerOfTwo)
            {
                std::cerr << "Warning: SamplingFrequency=" << configuration.acquisitionInput.samplingFrequencyHz
                          << " yields " << samplesPerCode << " samples per code period, which is not a power of "
                          << "two. The FFT-based acquisition/lookup-table path requires a power-of-two length and "
                          << "will fail for this configuration." << '\n';
            }
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid SamplingFrequency value in config, using default." << '\n';
        }
    }

    if (!m_configurationMap["AcquisitionMinimumDoppler"].empty())
    {
        try
        {
            configuration.acquisitionInput.acquisitionDopplerMinimum =
                std::stoi(m_configurationMap["AcquisitionMinimumDoppler"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid AcquisitionMinimumDoppler value in config, using default." << '\n';
        }
    }

    if (!m_configurationMap["AcquisitionMaximumDoppler"].empty())
    {
        try
        {
            configuration.acquisitionInput.acquisitionDopplerMaximum =
                std::stoi(m_configurationMap["AcquisitionMaximumDoppler"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid AcquisitionMaximumDoppler value in config, using default." << '\n';
        }
    }

    if (!m_configurationMap["AcquisitionDopplerSearchRange"].empty())
    {
        try
        {
            configuration.acquisitionInput.acquisitionDopplerSearchRange =
                std::stoi(m_configurationMap["AcquisitionDopplerSearchRange"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid AcquisitionDopplerSearchRange value in config, using default." << '\n';
        }
    }

    if (configuration.acquisitionInput.acquisitionDopplerSearchRange <= 0 ||
        configuration.acquisitionInput.acquisitionDopplerMinimum >=
            configuration.acquisitionInput.acquisitionDopplerMaximum)
    {
        std::cerr << "Invalid Doppler search configuration (range must be positive and minimum below maximum), "
                     "using defaults."
                  << '\n';
        const AcquisitionInput defaults{};
        configuration.acquisitionInput.acquisitionDopplerMinimum = defaults.acquisitionDopplerMinimum;
        configuration.acquisitionInput.acquisitionDopplerMaximum = defaults.acquisitionDopplerMaximum;
        configuration.acquisitionInput.acquisitionDopplerSearchRange = defaults.acquisitionDopplerSearchRange;
    }

    if (!m_configurationMap["AcquisitionCn0Threshold"].empty())
    {
        try
        {
            const double threshold = std::stod(m_configurationMap["AcquisitionCn0Threshold"]);
            if (!std::isfinite(threshold) || threshold <= 0.0)
            {
                throw std::invalid_argument("AcquisitionCn0Threshold out of range");
            }
            configuration.acquisitionInput.acquisitionCn0ThresholdDbHz = threshold;
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid AcquisitionCn0Threshold value in config, using default." << '\n';
        }
    }

    if (!m_configurationMap["TrackingTelemetryIntervalBlocks"].empty())
    {
        try
        {
            const int interval = std::stoi(m_configurationMap["TrackingTelemetryIntervalBlocks"]);
            if (interval < 1)
            {
                throw std::invalid_argument("TrackingTelemetryIntervalBlocks out of range");
            }
            configuration.trackingInput.telemetryIntervalBlocks = interval;
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid TrackingTelemetryIntervalBlocks value in config, using default." << '\n';
        }
    }

    if (!m_configurationMap["ProfilerEnabled"].empty())
    {
        try
        {
            configuration.profilerInput.enabled = (std::stoi(m_configurationMap["ProfilerEnabled"]) != 0) ? 1 : 0;
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid ProfilerEnabled value in config, using default." << '\n';
        }
    }

    if (!m_configurationMap["PllBandwidthHz"].empty())
    {
        try
        {
            configuration.trackingInput.pllBandwidthHz = std::stod(m_configurationMap["PllBandwidthHz"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid PllBandwidthHz value in config, using default." << '\n';
        }
    }

    if (!m_configurationMap["DllBandwidthHz"].empty())
    {
        try
        {
            configuration.trackingInput.dllBandwidthHz = std::stod(m_configurationMap["DllBandwidthHz"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid DllBandwidthHz value in config, using default." << '\n';
        }
    }

    if (!m_configurationMap["FllBandwidthHz"].empty())
    {
        try
        {
            configuration.trackingInput.fllBandwidthHz = std::stod(m_configurationMap["FllBandwidthHz"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid FllBandwidthHz value in config, using default." << '\n';
        }
    }

    if (!m_configurationMap["RateAidBandwidthHz"].empty())
    {
        try
        {
            configuration.trackingInput.rateAidBandwidthHz = std::stod(m_configurationMap["RateAidBandwidthHz"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid RateAidBandwidthHz value in config, using default." << '\n';
        }
    }

    if (!m_configurationMap["FllPullInBlocks"].empty())
    {
        try
        {
            configuration.trackingInput.fllPullInBlocks = std::stoi(m_configurationMap["FllPullInBlocks"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid FllPullInBlocks value in config, using default." << '\n';
        }
    }

    if (!m_configurationMap["FixOutputIntervalBlocks"].empty())
    {
        try
        {
            configuration.pvtSolverInput.fixOutputIntervalBlocks =
                std::stoi(m_configurationMap["FixOutputIntervalBlocks"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid FixOutputIntervalBlocks value in config, using default." << '\n';
        }
    }

    if (!m_configurationMap["PvtTropoEnabled"].empty())
    {
        try
        {
            configuration.pvtSolverInput.tropoEnabled = std::stoi(m_configurationMap["PvtTropoEnabled"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid PvtTropoEnabled value in config, using default." << '\n';
        }
    }

    if (!m_configurationMap["PvtElevationMaskDeg"].empty())
    {
        try
        {
            configuration.pvtSolverInput.elevationMaskDeg = std::stod(m_configurationMap["PvtElevationMaskDeg"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid PvtElevationMaskDeg value in config, using default." << '\n';
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
