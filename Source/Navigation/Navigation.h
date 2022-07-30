#ifndef INCLUDED_NAVIGATION_H
#define INCLUDED_NAVIGATION_H

#include <vector>
#include <QObject>
#include <QThread>
#include "../Logger/Logger.h"

namespace GPSOpenCl
{
    class Navigation : public QThread
    {
        Q_OBJECT
    public:
        Navigation(std::vector<double> navInput, QObject *parent = 0);
        ~Navigation();
        void run();
    private:
        void checkPreamble();
        std::vector<double> findPreamble(std::vector<double> input, std::vector<double> kernel);
        std::vector<double> m_input;
        Logger* m_logger;
    };
}

#endif // ! INCLUDED_NAVIGATION_H