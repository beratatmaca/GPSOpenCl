#include "GPSOpenClApplication.h"

#include <iostream>

using namespace GPSOpenCl;

Application::Application(Settings::Configuration conf) : m_configuration(conf)
{
    std::cout << SOFTWARE_NAME << " " << SOFTWARE_VERSION << " started to run" << std::endl;

    m_gpu = new Compute();

    m_code = new Code(m_configuration);
    m_code->createLookupTable(m_gpu);

    m_acquisition = new Acquisition(m_configuration);

    initializeChannels();
}

Application::~Application()
{
    delete m_gpu;
    delete m_acquisition;
}

void Application::searchForSatellites(const ComplexFloatVector input)
{
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        m_acquisition->correlate(input, m_gpu, m_code, &m_channels[i]);
        m_channels[i].checkAcquisition();
    }
}

void Application::trackSatellites(const ComplexFloatVector input)
{
}

void Application::initializeChannels()
{
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        m_channels[i].m_svId = i + 1;
    }
}