#ifndef INCLUDED_LOGGER_H
#define INCLUDED_LOGGER_H

#include <QFile>
#include <QIODevice>
#include <QTextStream>
#include <complex>

namespace GPSOpenCl
{
    class Logger
    {
    public:
        Logger();
        ~Logger();

        template <typename T>
        void log(T data)
        {
            if (!m_file->isOpen())
            {
                m_file->open(QIODevice::Append | QIODevice::Text);
            }
            *m_stream << data << "\n";
            m_stream->flush();
            m_file->close();
        }

        template <typename T>
        void log(T *data, int length)
        {
            if (!m_file->isOpen())
            {
                m_file->open(QIODevice::Append | QIODevice::Text);
            }
            for (int i = 0; i < length; i++)
            {
                *m_stream << data[i] << "\n";
            }
            m_stream->flush();
            m_file->close();
        }

        void log(std::vector<std::complex<double>> data);
        void log(std::vector<double> data);

    private:
        QFile *m_file;
        QTextStream *m_stream;
    };
}

#endif // ! INCLUDED_LOGGER_H