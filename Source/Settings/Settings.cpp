#include "Settings.h"

#include <cstring>

GPSOpenCl::Settings::Settings(/* args */)
{
}

GPSOpenCl::Settings::~Settings()
{
}

/**
 * @brief A function that reads an ini file
 * @param ini file path
 *
 * @return void
 *
 */
void GPSOpenCl::Settings::readIniFile(std::string iniFilePath)
{
    std::ifstream iniFile(iniFilePath, std::ios_base::in);
    if (!iniFile.is_open())
    {
        std::cout << "Could not open ini file" << std::endl;
    }
    else
    {
        std::string line;
        while (std::getline(iniFile, line))
        {
            std::cout << line << std::endl;
            // Get parameter
            std::string param = line.substr(0, line.find("="));
            // Get value
            std::string value = line.substr(line.find("=") + 1);
            // Check if param is the correct format
            // Extract white spaces from string param
            //param.erase(std::remove(param.begin(), param.end(), ' '), param.end());
            //line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
            // Check if param is empty
            // Set value to map
            setSetting(param, value);
        }
    }
    iniFile.close();
}

void GPSOpenCl::Settings::setSetting(std::string key, std::string value)
{
    m_settingsMap[key] = value;
}

void GPSOpenCl::Settings::getSetting(std::string key, std::string& value)
{
    value = m_settingsMap[key];
}
