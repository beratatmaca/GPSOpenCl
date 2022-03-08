#include "FileHandler.h"

GPSOpenCl::FileHandler::FileHandler()
{
    m_dataBuf = new char[BUF_SIZE];
}

GPSOpenCl::FileHandler::~FileHandler()
{
}

GPSOpenCl::FileHandler::readFile(Settings settings)
{
    bool success = false;
    std::string fileName = settings.getSetting("fileName");
    std::ifstream file(fileName, std::ios::in);
    if (!file.is_open())
    {
        std::cout << "File not found" << std::endl;
    }
    else
    {
        file.seekg (0, file.end);
        int length = file.tellg();
        file.seekg (0, file.beg);
        std::cout << "File length: " << length << std::endl;

        file.read (buffer,length);
        success = true;
    }
    return success;
}