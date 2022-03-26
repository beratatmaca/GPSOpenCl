#include "Settings.h"

GPSOpenCl::Settings::Settings()
{
    m_setting = new QSettings("./../../default.ini", QSettings::IniFormat);
    readSettings();
}

GPSOpenCl::Settings::~Settings()
{
}

void GPSOpenCl::Settings::readSettings()
{
    // m_setting->beginGroup("Settings");
    //  = defaultSettings->value("preambleLength", 160).toInt();;
}