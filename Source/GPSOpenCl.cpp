#include <QCoreApplication>
#include <QTimer>
#include <QDebug>

#include "IOManagement/FileHandler.h"
#include "Code/CACode.h"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    GPSOpenCl::FileHandler fileHandler("gpssim.bin");
    fileHandler.readFile();
    GPSOpenCl::CACode caCode;
    caCode.createCACodeTable();
    for (int i=0;i<100;i++)
    {
        qDebug() << caCode.m_data[0][i];
    }

    QTimer::singleShot(0, &app, &QCoreApplication::quit);
    return app.exec();
}