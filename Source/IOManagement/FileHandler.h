#ifndef INCLUDED_FILE_HANDLER_H
#define INCLUDED_FILE_HANDLER_H

#include <QString>
#include <QFile>
#include <QByteArray>
#include <complex>

#define NUM_OF_SAMPLES 4096000

namespace GPSOpenCl
{
    class FileHandler
    {
    public:
        FileHandler(QString fileName);
        ~FileHandler();

        void readFile();
        std::vector<std::complex<double>> m_data;

    private:
        QFile *m_rawDataFile;
        QByteArray m_dataBuffer;
    };
}

#endif // ! INCLUDED_FILE_HANDLER_H