#ifndef INCLUDED_SETTINGS_H
#define INCLUDED_SETTINGS_H

#include <QSettings>

namespace GPSOpenCl
{
    class Settings
    {
    public:
        Settings();
        ~Settings();
        void readSettings();
    private:
        QSettings* m_setting;
    };
}

#endif // ! INCLUDED_SETTINGS_H