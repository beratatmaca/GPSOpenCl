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

#include <chrono>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

int main(int argc, char **argv)
{
    GPSOpenCl::Settings settings;
    settings.captureSettings();

    std::string signalPath;
    if (argc > 1)
    {
        signalPath = argv[1];
    }
    else
    {
        const std::vector<std::string> candidateBinPaths = {
            "build/live_stream.bin", "../build/live_stream.bin", "live_stream.bin"};
        for (const auto &path : candidateBinPaths)
        {
            const std::ifstream f(path);
            if (f.good())
            {
                signalPath = path;
                break;
            }
        }
    }

    if (signalPath.empty())
    {
        const std::vector<std::string> candidatePaths = {"Tests/Scripts/inputSignal.txt",
                                                         "../Tests/Scripts/inputSignal.txt",
                                                         "../../Tests/Scripts/inputSignal.txt",
                                                         "Scripts/inputSignal.txt",
                                                         "inputSignal.txt"};

        for (const auto &path : candidatePaths)
        {
            const std::ifstream f(path);
            if (f.good())
            {
                signalPath = path;
                break;
            }
        }
    }

    auto compositeSink = std::make_shared<GPSOpenCl::CompositeSink>();
    compositeSink->addSink(std::make_shared<GPSOpenCl::FileSink>("build/telemetry_wire.log"));
#ifdef GPSOPENCL_ENABLE_ZMQ
    compositeSink->addSink(std::make_shared<GPSOpenCl::ZmqSink>("ipc:///tmp/gpsopencl/telemetry.sock"));
#endif
    auto sink = std::static_pointer_cast<GPSOpenCl::Sink>(compositeSink);

    std::shared_ptr<GPSOpenCl::Source> source;
    bool sourceInitialized = false;
    if (signalPath.find(".fifo") != std::string::npos)
    {
        auto simSource = std::make_shared<GPSOpenCl::GpsSdrSimSource>();
        GPSOpenCl::SourceInput srcInput = settings.configuration.sourceInput;
        snprintf(srcInput.fifoPath, sizeof(srcInput.fifoPath), "%s", signalPath.c_str());
        sourceInitialized = simSource->initialize(srcInput);
        source = simSource;
    }
    else
    {
        auto fileSource = std::make_shared<GPSOpenCl::FileSource>();
        GPSOpenCl::SourceInput srcInput = settings.configuration.sourceInput;
        snprintf(srcInput.fifoPath, sizeof(srcInput.fifoPath), "%s", signalPath.c_str());
        sourceInitialized = fileSource->initialize(srcInput);
        source = fileSource;
    }

    if (!sourceInitialized)
    {
        std::cerr << "Failed to initialize signal source (path: '" << signalPath << "'). Nothing to process, aborting."
                  << '\n';
        return 1;
    }

    source->setSink(sink);

    GPSOpenCl::Application app(settings.configuration);
    app.setSink(sink);

    struct SignalBlock
    {
        uint32_t blockIndex{0};
        GPSOpenCl::ComplexFloatVector samples;
    };

    GPSOpenCl::BoundedQueue<SignalBlock> blockQueue(16);
    GPSOpenCl::BoundedQueue<GPSOpenCl::ComplexFloatVector> recycleQueue(16);

    std::cout << "\n=============================================" << '\n';
    std::cout << "   GPS Processing Pipeline Starting (Real-Time Loop)" << '\n';
    std::cout << "=============================================" << '\n';

    std::thread producerThread(
        [&]()
        {
            try
            {
                uint32_t blockIdx = 0;
                while (true)
                {
                    SignalBlock block;
                    block.blockIndex = blockIdx;
                    recycleQueue.tryPop(block.samples);
                    GPSOpenCl::SourceOutput telemetry{};
                    const bool ok = source->readBlock(block.samples, telemetry);
                    if (!ok || block.samples.empty())
                    {
                        break;
                    }
                    if (!blockQueue.push(std::move(block)))
                    {
                        break;
                    }
                    blockIdx++;
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << "Producer thread terminated by exception: " << e.what() << '\n';
            }
            blockQueue.finish();
        });

    SignalBlock block;
    try
    {
        while (blockQueue.pop(block))
        {
            app.processBlock(block.samples, block.blockIndex);
            recycleQueue.tryPush(std::move(block.samples));
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Consumer loop terminated by exception: " << e.what() << '\n';
        blockQueue.finish();
    }

    if (producerThread.joinable())
    {
        producerThread.join();
    }

    std::cout << "\n=============================================" << '\n';
    std::cout << "   GPS Processing Pipeline Completed Successfully" << '\n';
    std::cout << "=============================================" << '\n';

    return 0;
}
