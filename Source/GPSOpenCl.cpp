#include <QCoreApplication>
#include <QTimer>
#include <QDebug>
#include <QFile>
#include <QTextStream>

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

    // GPSOpenCl::Acquisition acquisition(caCode.m_code.at(0), fileHandler.m_data, 3.0);
    // acquisition.start();
    // acquisition.wait();

    // GPSOpenCl::Tracking tracking(caCode.m_code.at(0), fileHandler.m_data);
    // tracking.start();
    // tracking.wait();

    // Read the file again
    QFile file("./../../IPs.txt");
    file.open(QIODevice::ReadOnly);
    std::vector<double> trackResults(36000);
    QTextStream in(&file);
    QStringList trackResultsStr = in.readAll().split('\n');
    for(int i=0;i<36000;i++)
    {
        trackResults.at(i) = trackResultsStr.at(i).toDouble();
    }
    GPSOpenCl::Navigation nav(trackResults);
    nav.start();
    nav.wait();

    QTimer::singleShot(0, &app, &QCoreApplication::quit);
    return app.exec();
}