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
        void setSetting(std::string key, std::string value);
        void getSetting(std::string key, std::string& value);
    private:
        std::map<std::string, std::string> m_settingsMap;
    };
}

#endif // INCLUDED_SETTINGS_H
