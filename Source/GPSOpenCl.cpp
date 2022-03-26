#include <QCoreApplication>
#include <QTimer>
#include <QDebug>

#define CL_TARGET_OPENCL_VERSION 220

#include "IOManagement/FileHandler.h"
#include "Code/CACode.h"
#include "Utils/Utils.h"
#include "Acquisition/Acquisition.h"
#include "Tracking/Tracking.h"
#include "Navigation/Navigation.h"


int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    GPSOpenCl::FileHandler fileHandler("gpssim.bin");
    fileHandler.readFile();

    GPSOpenCl::CACode caCode;

    GPSOpenCl::Acquisition acquisition(caCode.m_code.at(0), fileHandler.m_data, 3.0);
    acquisition.start();
    acquisition.wait();

    GPSOpenCl::Tracking tracking(caCode.m_code.at(0), fileHandler.m_data);
    tracking.start();
    tracking.wait();

    GPSOpenCl::Navigation nav(tracking.Ip);
    nav.start();
    nav.wait();

    QTimer::singleShot(0, &app, &QCoreApplication::quit);
    return app.exec();
}