#include "Logger.h"

#include <QDebug>
#include <QDir>

GPSOpenCl::Logger::Logger()
{
    QString fileName = "log.txt";
    m_file = new QFile(QDir::currentPath() + "/../../" + fileName);
    if (!m_file->open(QIODevice::WriteOnly | QIODevice::Text))
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