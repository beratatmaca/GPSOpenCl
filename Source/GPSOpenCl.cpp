#include "Settings/Settings.h"

int main()
{
    GPSOpenCl::Settings settings;
    settings.readIniFile("Settings/Settings.ini");
    return 0;
}