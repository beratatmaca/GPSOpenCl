#include "Settings/Settings.h"

int main()
{
    GPSOpenCl::Settings settings;
    settings.readIniFile("../../Source/default.ini");
    return 0;
}