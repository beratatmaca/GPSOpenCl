#include "Settings.h"

Settings::Settings(/* args */)
{
}

Settings::~Settings()
{
}

/**
 * @brief A function that reads an ini file
 * @param ini file path
 * 
 * @return void 
 * 
 */
void Settings::readIniFile(std::string iniFilePath)
{
    std::ifstream iniFile(iniFilePath);
    if (!iniFile.is_open())
    {
        std::cout << "Could not open ini file" << std::endl;
        return;
    }
    std::string line;
    while (std::getline(iniFile, line))
    {
        std::cout << line << std::endl;
        // Get parameter
        std::string param = line.substr(0, line.find("="));
        // Get value
        std::string value = line.substr(line.find("=") + 1);
        // Set parameter
        if (param == "parameter")
        {
            std::cout << "Setting parameter to " << value << std::endl;
        }
    }
    iniFile.close();
}