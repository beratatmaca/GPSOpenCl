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

void GPSOpenCl::Logger::log(std::vector<std::complex<double>> data)
{
    if (!m_file->isOpen())
    {
        m_file->open(QIODevice::Append | QIODevice::Text);
    }
    for (auto it = data.begin(); it != data.end(); ++it)
    {
        *m_stream << std::real(*it) << "+" << std::imag(*it) << "i"
                  << "\n";
    }
    m_stream->flush();
    m_file->close();
}

void GPSOpenCl::Logger::log(std::vector<double> data)
{
    if (!m_file->isOpen())
    {
        m_file->open(QIODevice::Append | QIODevice::Text);
    }
    for (auto it = data.begin(); it != data.end(); ++it)
    {
        *m_stream << *it << "\n";
    }
    m_stream->flush();
    m_file->close();
}
