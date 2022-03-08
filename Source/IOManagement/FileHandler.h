#ifndef INCLUDED_FILE_HANDLER_H
#define INCLUDED_FILE_HANDLER_H

#include "../Settings/Settings.h"

namespace GPSOpenCl
{
    class FileHandler
    {
    public:
        FileHandler();
        ~FileHandler();

        bool readFile(Settings settings);

        char* m_dataBuffer;
    private:
        std::string m_fileName;
        std::string 
    };
}

#endif // ! INCLUDED_FILE_HANDLER_H