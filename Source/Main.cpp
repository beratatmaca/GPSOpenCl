#include "GPSOpenClApplication.h"
#include "GPSOpenClCommon.h"
#include "GPSOpenClNmeaGenerator.h"
#include "GPSOpenClPVTSolver.h"
#include "GPSOpenClSettings.h"
#include "../Tests/TestUtils.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

int main(int argc, char **argv)
{
    GPSOpenCl::Settings settings;
    settings.captureSettings();

    GPSOpenCl::ComplexFloatVector fullInput;
    std::string signalPath = "";

    if (argc > 1)
    {
        signalPath = argv[1];
    }
    else
    {
        std::vector<std::string> candidateBinPaths = {
            "build/live_stream.bin",
            "../build/live_stream.bin",
            "live_stream.bin"
        };
        for (const auto &path : candidateBinPaths)
        {
            std::ifstream f(path);
            if (f.good())
            {
                signalPath = path;
                break;
            }
        }
    }

    if (!signalPath.empty() && signalPath.find(".bin") != std::string::npos)
    {
        std::cout << "Reading simulated binary IQ file: " << signalPath << std::endl;
        GPSOpenClTest::TestUtils::readFromFileBinaryIQ8(signalPath.c_str(), &fullInput);
    }

    if (fullInput.empty())
    {
        std::vector<std::string> candidatePaths = {
            "Tests/Scripts/inputSignal.txt",
            "../Tests/Scripts/inputSignal.txt",
            "../../Tests/Scripts/inputSignal.txt",
            "Scripts/inputSignal.txt",
            "inputSignal.txt"
        };

        for (const auto &path : candidatePaths)
        {
            GPSOpenClTest::TestUtils::readFromFileComplex(path.c_str(), &fullInput);
            if (!fullInput.empty())
            {
                std::cout << "Loaded text input signal from: " << path << std::endl;
                break;
            }
        }
    }

    int blockLength = settings.configuration.rawDataSettings.numberOfSamplesPerCode;
    if (blockLength <= 0) blockLength = 4096;

    size_t totalSamples = fullInput.size();
    size_t totalBlocks = totalSamples / blockLength;

    std::cout << "Loaded " << totalSamples << " samples (" << totalBlocks << " full 1-ms blocks)." << std::endl;

    if (totalBlocks == 0)
    {
        std::cerr << "Not enough data samples to process." << std::endl;
        return -1;
    }

    GPSOpenCl::Application app(settings.configuration);

    // Block 0: Satellite Acquisition Phase
    std::cout << "\n=============================================" << std::endl;
    std::cout << "   GPS Acquisition Phase (Block 0 / " << totalBlocks << ")" << std::endl;
    std::cout << "=============================================" << std::endl;

    GPSOpenCl::ComplexFloatVector acqBlock(fullInput.begin(), fullInput.begin() + blockLength);
    app.searchForSatellites(acqBlock);

    // Initial Telemetry Export right after acquisition
    GPSOpenCl::ReceiverPvtSolution solution;
    app.computeNavigationSolution(solution);

    // Blocks 0..N-1: Continuous Live Tracking Phase
    std::cout << "\n=============================================" << std::endl;
    std::cout << "   GPS Multi-Block Tracking & Live Streaming Phase (" << totalBlocks << " Blocks)" << std::endl;
    std::cout << "=============================================" << std::endl;

    for (size_t b = 0; b < totalBlocks; b++)
    {
        auto blockStart = fullInput.begin() + (b * blockLength);
        auto blockEnd = blockStart + blockLength;
        GPSOpenCl::ComplexFloatVector currentBlock(blockStart, blockEnd);

        app.trackSatellites(currentBlock);

        // Update telemetry JSON every 50 blocks (~50 ms real time) to stream live to Plotly Dashboard
        if (b % 50 == 0 || b == totalBlocks - 1)
        {
            app.computeNavigationSolution(solution);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    std::cout << "\n=============================================" << std::endl;
    std::cout << "   GPS Processing Pipeline Completed Successfully" << std::endl;
    std::cout << "=============================================" << std::endl;

    return 0;
}