#include "Logger.h"

#include <QDebug>

GPSOpenCl::Logger::Logger()
{
    QString fileName = "log.txt";
    m_file = new QFile("/../../"+fileName);
    if (!m_file->open(QIODevice::Append | QIODevice::Text))
    {
        qDebug() << "Could not open file for writing";
    }
    m_stream = new QTextStream(m_file);
}

GPSOpenCl::Logger::~Logger()
{
    m_file->close();
    delete m_file;
}