#ifndef INCLUDED_SETTINGS_H
#define INCLUDED_SETTINGS_H

#include <cstring>
#include <iostream>
#include <fstream>
#include <map>

namespace GPSOpenCl
{
    class Settings
    {
    private:
        /* data */
    public:
        Settings(/* args */);
        ~Settings();
        void readIniFile(std::string iniFilePath);
    private:
        void setSettings(std::string param, std::string value);
        // Create a map to hold the settings
        std::map<std::string, std::string> settingsMap;

    };
}

#endif // INCLUDED_SETTINGS_H
