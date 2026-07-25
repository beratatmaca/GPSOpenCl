#include "GPSOpenClApplication.h"
#include "GPSOpenClBoundedQueue.h"
#include "GPSOpenClCommon.h"
#include "GPSOpenClFileSink.h"
#include "GPSOpenClFileSource.h"
#include "GPSOpenClGpsSdrSimSource.h"
#include "GPSOpenClNmeaGenerator.h"
#include "GPSOpenClPVTSolver.h"
#include "GPSOpenClSettings.h"
#include "GPSOpenClZmqSink.h"
#include "../Tests/TestUtils.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

int main(int argc, char **argv)
{
    GPSOpenCl::Settings settings;
    settings.captureSettings();

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

    if (signalPath.empty())
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
            std::ifstream f(path);
            if (f.good())
            {
                signalPath = path;
                break;
            }
        }
    }

    // 1. Create Sink Publisher
#ifdef GPSOPENCL_ENABLE_ZMQ
    auto sink = std::make_shared<GPSOpenCl::ZmqSink>("ipc:///tmp/gpsopencl/telemetry.sock");
#else
    auto sink = std::make_shared<GPSOpenCl::FileSink>("build/telemetry_wire.log");
#endif

    // 2. Initialize Source
    std::shared_ptr<GPSOpenCl::Source> source;
    if (signalPath.find(".fifo") != std::string::npos)
    {
        auto simSource = std::make_shared<GPSOpenCl::GpsSdrSimSource>();
        GPSOpenCl::SourceInput srcInput = settings.configuration.sourceInput;
        snprintf(srcInput.fifoPath, sizeof(srcInput.fifoPath), "%s", signalPath.c_str());
        simSource->initialize(srcInput);
        source = simSource;
    }
    else
    {
        auto fileSource = std::make_shared<GPSOpenCl::FileSource>();
        GPSOpenCl::SourceInput srcInput = settings.configuration.sourceInput;
        snprintf(srcInput.fifoPath, sizeof(srcInput.fifoPath), "%s", signalPath.c_str());
        fileSource->initialize(srcInput);
        source = fileSource;
    }
    source->setSink(sink);

    // 3. Application instance
    GPSOpenCl::Application app(settings.configuration);
    app.setSink(sink);
    app.setSource(source);

    // 4. Concurrency Model: Bounded Queue between Producer (Source) and Consumer (Application)
    struct SignalBlock
    {
        uint32_t blockIndex;
        GPSOpenCl::ComplexFloatVector samples;
    };

    GPSOpenCl::BoundedQueue<SignalBlock> blockQueue(16);

    std::cout << "\n=============================================" << std::endl;
    std::cout << "   GPS Processing Pipeline Starting (Real-Time Loop)" << std::endl;
    std::cout << "=============================================" << std::endl;

    // Producer Thread
    std::thread producerThread([&]() {
        uint32_t blockIdx = 0;
        while (true)
        {
            SignalBlock block;
            block.blockIndex = blockIdx;
            GPSOpenCl::SourceOutput telemetry{};
            bool ok = source->readBlock(block.samples, telemetry);
            if (!ok || block.samples.empty())
            {
                break;
            }
            if (!blockQueue.push(block))
            {
                break;
            }
            blockIdx++;
        }
        blockQueue.finish();
    });

    // Consumer Loop
    SignalBlock block;
    while (blockQueue.pop(block))
    {
        app.processBlock(block.samples, block.blockIndex);
    }

    if (producerThread.joinable())
    {
        producerThread.join();
    }

    std::cout << "\n=============================================" << std::endl;
    std::cout << "   GPS Processing Pipeline Completed Successfully" << std::endl;
    std::cout << "=============================================" << std::endl;

    return 0;
}