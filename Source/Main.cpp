#include "GPSOpenClSettings.h"
#include "Source/GPSOpenClApplication.h"

#include "../Tests/TestUtils.h"
#include "Source/GPSOpenClCommon.h"

int main()
{
    GPSOpenCl::ComplexFloatVector input;
    GPSOpenClTest::TestUtils::readFromFileComplex("../../Tests/Scripts/inputSignal.txt", &input);

    auto start = input.begin();
    auto end = input.begin() + 4096;
    GPSOpenCl::ComplexFloatVector vector(start, end);

    GPSOpenCl::Settings settings;
    settings.captureSettings();

    GPSOpenCl::Application app(settings.configuration);
    app.searchForSatellites(vector);

    return 0;
}