#include "Settings.h"

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
    std::ifstream iniFile(iniFilePath);
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
            if (param.find(" ") != std::string::npos)
            {
                std::cout << "Error: Parameter contains spaces" << std::endl;
                return;
            }
            // Set value to map
            setSettings(param, value);
        }
    }
    iniFile.close();
}

void GPSOpenCl::Settings::setSettings(std::string param, std::string value)
{
    settingsMap[param] = value;
}
