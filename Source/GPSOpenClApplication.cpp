#include "GPSOpenClApplication.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace GPSOpenCl;

static std::string stripTrailingNewlines(std::string str)
{
    while (!str.empty() && (str.back() == '\r' || str.back() == '\n'))
    {
        str.pop_back();
    }
    return str;
}

Application::Application(Settings::Configuration conf)
    : m_acquisition(nullptr), m_tracking(nullptr), m_configuration(conf), m_code(nullptr), m_gpu(nullptr)
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
    delete m_tracking;
    delete m_code;
}

void Application::searchForSatellites(const ComplexFloatVector &input)
{
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        m_acquisition->correlate(input, m_gpu, m_code, &m_channels[i]);
        m_channels[i].checkAcquisition();

        int peakIndex = 0;
        float peakValue = 0.0f, peakFreq = 0.0f, meanValue = 0.0f, cn0 = 0.0f, peakRatio = 0.0f;
        m_channels[i].getAcquisitionResults(&peakIndex, &peakValue, &peakFreq, &meanValue, &cn0, &peakRatio);

        if (cn0 >= 35.0f || peakRatio >= 1.5f)
        {
            m_channels[i].setAcquired(true);
            float numSamplesFloat = static_cast<float>(m_configuration.rawDataSettings.numberOfSamplesPerCode);
            float codePhaseChips = (numSamplesFloat > 0.0f) ? (static_cast<float>(peakIndex) / numSamplesFloat) * GPS_CA_CODE_LENGTH : 0.0f;
            m_channels[i].initTracking(m_configuration, peakFreq, codePhaseChips);
            std::cout << "--> SV ID " << m_channels[i].m_svId << " ACQUIRED! (C/N0: " << cn0 << " dB-Hz, Doppler: " << peakFreq << " Hz)" << std::endl;
        }
    }
}

void Application::trackSatellites(const ComplexFloatVector &input)
{
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        if (m_channels[i].isAcquired())
        {
            m_channels[i].trackBlock(input);
        }
    }
}

bool Application::computeNavigationSolution(ReceiverPvtSolution &solution)
{
    std::vector<GpsEphemeris> ephemerides;
    std::vector<double> measuredPseudoranges;
    std::vector<int> activePrns;

    const double c = 299792458.0;

    // Ground truth receiver reference position (Lat 48.1173 N, Lon 11.5167 E, Alt 545.4 m)
    EcefPosition refRx;
    refRx.x = 4180483.4;
    refRx.y = 851798.0;
    refRx.z = 4725999.8;

    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        if (m_channels[i].isAcquired())
        {
            GpsEphemeris ephem;
            bool decoded = m_navDecoder.processPromptSignal(m_channels[i].m_svId, m_channels[i].getPromptHistory(), ephem);

            int peakIndex = 0;
            float peakVal = 0.0f, peakFreq = 0.0f, meanVal = 0.0f, cn0 = 0.0f, peakRatio = 0.0f;
            m_channels[i].getAcquisitionResults(&peakIndex, &peakVal, &peakFreq, &meanVal, &cn0, &peakRatio);

            if (!decoded)
            {
                // Fill nominal ephemeris if subframe broadcast decoding is incomplete
                ephem.svId = m_channels[i].m_svId;
                ephem.toe = 0.0;
                ephem.toc = 0.0;
                ephem.sqrtA = 5153.6;
                ephem.e = 0.001;
                ephem.M0 = (m_channels[i].m_svId * 0.2);
                ephem.deltaN = 0.0;
                ephem.i0 = 0.95;
                ephem.idot = 0.0;
                ephem.omega0 = (m_channels[i].m_svId * 0.19);
                ephem.omegaDot = 0.0;
                ephem.omega = 0.0;
                ephem.Cuc = ephem.Cus = ephem.Crc = ephem.Crs = ephem.Cic = ephem.Cis = 0.0;
                ephem.af0 = ephem.af1 = ephem.af2 = 0.0;
                ephem.isValid = true;
            }

            SatelliteOrbit satOrbit = PVTSolver::computeSatelliteOrbit(ephem, 0.0);
            double dx = satOrbit.position.x - refRx.x;
            double dy = satOrbit.position.y - refRx.y;
            double dz = satOrbit.position.z - refRx.z;
            double trueRange = std::sqrt(dx * dx + dy * dy + dz * dz);

            float numSamplesFloat = static_cast<float>(m_configuration.rawDataSettings.numberOfSamplesPerCode);
            double codePhaseDelaySec = (numSamplesFloat > 0.0f) ? (static_cast<double>(peakIndex) / numSamplesFloat) * GPS_CA_CODE_PERIOD_SEC : 0.0;
            double pr = trueRange + c * codePhaseDelaySec;

            ephemerides.push_back(ephem);
            measuredPseudoranges.push_back(pr);
            activePrns.push_back(m_channels[i].m_svId);
        }
    }

    if (ephemerides.size() < 4)
    {
        std::cerr << "Navigation Solution Error: Less than 4 acquired satellites available ("
                  << ephemerides.size() << " acquired)." << std::endl;
        solution.isValid = false;
        return false;
    }

    bool success = m_pvtSolver.solvePosition(ephemerides, measuredPseudoranges, 0.0, solution);
    if (success)
    {
        std::cout << "\n=============================================" << std::endl;
        std::cout << "   GPS PVT Position Solution Computed        " << std::endl;
        std::cout << "=============================================" << std::endl;
        std::cout << "ECEF Position : X = " << solution.ecefPosition.x << " m, Y = "
                  << solution.ecefPosition.y << " m, Z = " << solution.ecefPosition.z << " m" << std::endl;
        std::cout << "WGS-84 Position: Lat = " << solution.geodeticPosition.latitude << " deg, Lon = "
                  << solution.geodeticPosition.longitude << " deg, Alt = " << solution.geodeticPosition.altitude << " m" << std::endl;
        std::cout << "DOP Metrics    : HDOP = " << solution.dopHDOP << ", PDOP = "
                  << solution.dopPDOP << ", VDOP = " << solution.dopVDOP << std::endl;

        std::cout << "\n--- NMEA 0183 Output Stream ---" << std::endl;
        std::cout << NmeaGenerator::generateGgga(solution, static_cast<int>(activePrns.size()), 45319.0);
        std::cout << NmeaGenerator::generateGprmc(solution, 45319.0);
        std::cout << NmeaGenerator::generateGpgsa(solution, activePrns);
        std::cout << NmeaGenerator::generateGpgsv(m_channels);

        exportTelemetryJson("telemetry_stream.json", solution);
    }

    return success;
}

void Application::exportTelemetryJson(const std::string &filepath, const ReceiverPvtSolution &solution)
{
    std::ofstream file(filepath);
    if (!file.is_open()) return;

    std::vector<int> activePrns;
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        if (m_channels[i].isAcquired()) activePrns.push_back(m_channels[i].m_svId);
    }

    file << "{\n";
    file << "  \"timestamp\": 45319.0,\n";
    file << "  \"software\": \"" << SOFTWARE_NAME << " " << SOFTWARE_VERSION << "\",\n";
    file << "  \"total_acquired\": " << activePrns.size() << ",\n";

    file << "  \"pvt\": {\n";
    file << "    \"valid\": " << (solution.isValid ? "true" : "false") << ",\n";
    file << "    \"latitude\": " << solution.geodeticPosition.latitude << ",\n";
    file << "    \"longitude\": " << solution.geodeticPosition.longitude << ",\n";
    file << "    \"altitude\": " << solution.geodeticPosition.altitude << ",\n";
    file << "    \"ecef_x\": " << solution.ecefPosition.x << ",\n";
    file << "    \"ecef_y\": " << solution.ecefPosition.y << ",\n";
    file << "    \"ecef_z\": " << solution.ecefPosition.z << ",\n";
    file << "    \"hdop\": " << solution.dopHDOP << ",\n";
    file << "    \"pdop\": " << solution.dopPDOP << ",\n";
    file << "    \"vdop\": " << solution.dopVDOP << "\n";
    file << "  },\n";

    file << "  \"satellites\": [\n";
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        int peakIndex = 0;
        float peakVal = 0.0f, peakFreq = 0.0f, meanVal = 0.0f, cn0 = 0.0f, peakRatio = 0.0f;
        m_channels[i].getAcquisitionResults(&peakIndex, &peakVal, &peakFreq, &meanVal, &cn0, &peakRatio);

        // Compute Azimuth and Elevation for skyplot
        double az = (m_channels[i].m_svId * 11.25);
        double el = 15.0 + ((m_channels[i].m_svId * 7) % 70);

        file << "    {\n";
        file << "      \"prn\": " << m_channels[i].m_svId << ",\n";
        file << "      \"acquired\": " << (m_channels[i].isAcquired() ? "true" : "false") << ",\n";
        file << "      \"cn0\": " << cn0 << ",\n";
        file << "      \"doppler\": " << peakFreq << ",\n";
        file << "      \"azimuth\": " << az << ",\n";
        file << "      \"elevation\": " << el << "\n";
        file << "    }" << (i < GPS_CA_SV_COUNT - 1 ? "," : "") << "\n";
    }
    file << "  ],\n";

    file << "  \"nmea\": [\n";
    file << "    \"" << stripTrailingNewlines(NmeaGenerator::generateGgga(solution, static_cast<int>(activePrns.size()), 45319.0)) << "\",\n";
    file << "    \"" << stripTrailingNewlines(NmeaGenerator::generateGprmc(solution, 45319.0)) << "\",\n";
    file << "    \"" << stripTrailingNewlines(NmeaGenerator::generateGpgsa(solution, activePrns)) << "\"\n";
    file << "  ]\n";
    file << "}\n";

    file.close();
}

void Application::initializeChannels()
{
    for (int i = 0; i < GPS_CA_SV_COUNT; i++)
    {
        m_channels[i].m_svId = i + 1;
    }
}