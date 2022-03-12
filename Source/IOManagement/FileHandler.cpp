#include "FileHandler.h"

#include <QDebug> // for qDebug()
#include <QDir> // for QDir::currentPath()

GPSOpenCl::FileHandler::FileHandler(QString fileName)
{
    m_rawDataFile = new QFile(QDir::currentPath() + "/../../" + fileName);
    qDebug() << "FileHandler: " << QDir::currentPath() + "/../../" + fileName;
    if(!m_rawDataFile->exists())
    {
        qDebug() << "FileHandler:" << "File does not exist";
    }
}

GPSOpenCl::FileHandler::~FileHandler()
{
    delete m_rawDataFile;
}

void GPSOpenCl::FileHandler::readFile()
{
    m_rawDataFile->open(QIODevice::ReadOnly);
    if (m_rawDataFile->isOpen())
    {
        m_dataBuffer = m_rawDataFile->read(NUM_OF_SAMPLES * 2); // * sizeof(data)
        for (int i = 0; i < m_dataBuffer.size(); i+=2)
        {
            m_data[i/2] = std::complex<double>(static_cast<double>(m_dataBuffer.at(i)), \
                                                static_cast<double>(m_dataBuffer.at(i+1)));
        }
        qDebug() << "FileHandler: " << m_dataBuffer.size();
        m_rawDataFile->close();
    }
    else
    {
        qDebug() << "FileHandler:" << "File is not open";
    }
}