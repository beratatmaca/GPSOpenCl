#ifndef INCLUDED_LOGGER_H
#define INCLUDED_LOGGER_H

#include <QFile>
#include <QIODevice>
#include <QTextStream>

namespace GPSOpenCl
{
    class Logger
    {
    public:
        Logger();
        ~Logger();

        template<typename T>
        void log(T data)
        {
            if (m_file->isOpen())
            {
                *m_stream << data << "\n";
            }
            m_file->close();
        }

        template<typename T>
        void log(T* data, int length)
        {
            if (m_file->isOpen())
            {
                for (int i = 0; i < length; i++)
                {
                    *m_stream << data[i] << "\n";
                }
            }
            m_file->close();
        }
        
    private:
        QFile* m_file;
        QTextStream* m_stream;
    };
}

#endif // ! INCLUDED_LOGGER_H