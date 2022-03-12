#ifndef INCLUDED_FILE_HANDLER_H
#define INCLUDED_FILE_HANDLER_H

#include <QString>
#include <QFile>
#include <QByteArray>
#include <complex>

#define NUM_OF_SAMPLES 163840

namespace GPSOpenCl
{
    class FileHandler
    {
    public:
        FileHandler(QString fileName);
        ~FileHandler();

        void readFile();
        std::complex<double> m_data[NUM_OF_SAMPLES] = {0};

    private:
        QFile *m_rawDataFile;
        QByteArray m_dataBuffer;
    };
}

#endif // ! INCLUDED_FILE_HANDLER_H