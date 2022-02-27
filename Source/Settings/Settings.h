#ifndef INCLUDED_SETTINGS_H
#define INCLUDED_SETTINGS_H

#include <cstring>
#include <iostream>
#include <fstream>

class Settings
{
private:
    /* data */
public:
    Settings(/* args */);
    ~Settings();
    void readIniFile(std::string iniFilePath);
};

#endif // INCLUDED_SETTINGS_H
