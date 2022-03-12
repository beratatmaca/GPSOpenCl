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
    int prn = 1;
    std::complex<double>* arr = caCode.conjFFTcode(caCode.m_code[prn - 1], 4096);

    QTimer::singleShot(0, &app, &QCoreApplication::quit);
    return app.exec();
}