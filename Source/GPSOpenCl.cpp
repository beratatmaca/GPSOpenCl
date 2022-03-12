#include <QCoreApplication>
#include <QTimer>
#include <QDebug>

#include "IOManagement/FileHandler.h"
#include "Code/CACode.h"
#include "Utils/Utils.h"
#include "Acquisition/Acquisition.h"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    GPSOpenCl::FileHandler fileHandler("gpssim.bin");
    fileHandler.readFile();

    GPSOpenCl::CACode caCode;
    caCode.createCACodeTable();

    GPSOpenCl::Acquisition acquisition(caCode.m_code[0], fileHandler.m_data[0]);
    acquisition.start();

    QTimer::singleShot(0, &app, &QCoreApplication::quit);
    return app.exec();
}