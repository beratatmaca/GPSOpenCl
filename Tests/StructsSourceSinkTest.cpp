#include "gtest/gtest.h"

#include <cstdio>
#include <fstream>

#include "GPSOpenClFileSink.h"
#include "GPSOpenClFileSource.h"
#include "GPSOpenClGpsSdrSimSource.h"
#include "GPSOpenClProfiler.h"
#include "GPSOpenClSink.h"
#include "GPSOpenClStructs.h"
#include "GPSOpenClZmqSink.h"

using namespace GPSOpenCl;

TEST(StructsTest, WireFormatAndPacking)
{
    SourceInput input{};
    EXPECT_EQ(input.structVersion, STRUCT_VERSION_1);

    AcquisitionOutput acqOut{};
    EXPECT_EQ(acqOut.structVersion, STRUCT_VERSION_2);
    EXPECT_EQ(acqOut.correlateMs, 0.0);

    TrackingOutput trackOut{};
    EXPECT_EQ(trackOut.structVersion, STRUCT_VERSION_2);
    EXPECT_EQ(trackOut.channelState, 0u);
    EXPECT_EQ(trackOut.carrierLockIndicator, 0.0);
    EXPECT_EQ(trackOut.codeLockRatio, 0.0);
    EXPECT_EQ(trackOut.correlatorTimeMs, 0.0);

    NavDecoderOutput navOut{};
    EXPECT_EQ(navOut.structVersion, STRUCT_VERSION_1);

    PvtSolverOutput pvtOut{};
    EXPECT_EQ(pvtOut.structVersion, STRUCT_VERSION_2);
    EXPECT_EQ(pvtOut.satellitesUsed, 0u);
    EXPECT_EQ(pvtOut.maxResidualMeters, 0.0);

    AtmosphericOutput atmOut{};
    EXPECT_EQ(atmOut.structVersion, STRUCT_VERSION_1);

    NmeaGeneratorOutput nmeaOut{};
    EXPECT_EQ(nmeaOut.structVersion, STRUCT_VERSION_1);

    ProfilerOutput profOut{};
    EXPECT_EQ(profOut.structVersion, STRUCT_VERSION_1);
}

TEST(StructsTest, WireSizesMatchDashboardParsers)
{
    EXPECT_EQ(sizeof(SourceOutput), 24u);
    EXPECT_EQ(sizeof(AcquisitionOutput), 64u);
    EXPECT_EQ(sizeof(TrackingOutput), 116u);
    EXPECT_EQ(sizeof(NavDecoderOutput), 196u);
    EXPECT_EQ(sizeof(PvtSolverOutput), 116u);
    EXPECT_EQ(sizeof(AtmosphericOutput), 40u);
    EXPECT_EQ(sizeof(ProfilerOutput), 88u);
}

TEST(FileSourceTest, SkipsMalformedTextLinesInsteadOfAborting)
{
    const char *path = "malformed_samples_test.txt";
    {
        std::ofstream file(path);
        file << "# recorded 2026-08-05\n";
        file << "1.0\n";
        file << "-1.0\n";
        file << "garbage\n";
        file << "0.5\n";
        file << "0.25\n";
    }

    FileSource source;
    SourceInput input{};
    snprintf(input.fifoPath, sizeof(input.fifoPath), "%s", path);
    input.samplingRate = 4096000.0;

    EXPECT_TRUE(source.initialize(input));
    std::remove(path);
}

TEST(FileSourceTest, FailsCleanlyWhenNothingParses)
{
    const char *path = "all_garbage_test.txt";
    {
        std::ofstream file(path);
        file << "header\njunk\nmore junk\n";
    }

    FileSource source;
    SourceInput input{};
    snprintf(input.fifoPath, sizeof(input.fifoPath), "%s", path);
    input.samplingRate = 4096000.0;

    EXPECT_FALSE(source.initialize(input));
    std::remove(path);
}

TEST(FileSinkTest, WritesLengthPrefixedRecordsReadableByTheDashboardParser)
{
    const char *path = "filesink_framing_test.log";
    std::remove(path);
    {
        FileSink sink(path);
        SourceOutput srcOut{};
        srcOut.blockIndex = 42;
        sink.publishSourceOutput(srcOut);
    }

    std::ifstream file(path, std::ios::binary);
    ASSERT_TRUE(file.is_open());

    uint32_t nameLen = 0;
    file.read(reinterpret_cast<char *>(&nameLen), sizeof(nameLen));
    ASSERT_EQ(nameLen, 12u);

    std::string name(nameLen, '\0');
    file.read(name.data(), nameLen);
    EXPECT_EQ(name, "SourceOutput");

    uint32_t dataLen = 0;
    file.read(reinterpret_cast<char *>(&dataLen), sizeof(dataLen));
    ASSERT_EQ(dataLen, sizeof(SourceOutput));

    SourceOutput decoded{};
    file.read(reinterpret_cast<char *>(&decoded), sizeof(decoded));
    ASSERT_TRUE(file.good());
    EXPECT_EQ(decoded.blockIndex, 42u);

    file.close();
    std::remove(path);
}

TEST(FileSinkTest, UnwritablePathIsHandledWithoutCrashing)
{
    FileSink sink("/nonexistent_gpsopencl_dir/wire.log");
    SourceOutput srcOut{};
    sink.publishSourceOutput(srcOut);
}

TEST(ZmqSinkTest, FailedBindDropsMessagesWithoutHangingOrCrashing)
{
    ZmqSink sink("bogus-transport://nowhere");
    SourceOutput srcOut{};
    sink.publishSourceOutput(srcOut);
}

class TestSubscriberSink : public Sink
{
  public:
    int publishedCount = 0;
    std::string lastIdentifier;

    void publish(const std::string &identifier, const void *, size_t) override
    {
        publishedCount++;
        lastIdentifier = identifier;
    }
};

TEST(SinkTest, CustomSubscriberSink)
{
    TestSubscriberSink sink;
    SourceOutput srcOut{};
    sink.publishSourceOutput(srcOut);
    EXPECT_EQ(sink.publishedCount, 1);
    EXPECT_EQ(sink.lastIdentifier, "SourceOutput");

    AcquisitionOutput acqOut{};
    sink.publishAcquisitionOutput(acqOut);
    EXPECT_EQ(sink.publishedCount, 2);
    EXPECT_EQ(sink.lastIdentifier, "AcquisitionOutput");
}

TEST(ProfilerTest, MeasureStageTime)
{
    Profiler profiler;
    auto testSink = std::make_shared<TestSubscriberSink>();
    profiler.setSink(testSink);

    profiler.startBlock(1, 0.001);
    {
        Profiler::ScopedTimer timer(profiler, Profiler::Stage::Acquisition);
    }
    ProfilerOutput profOut = profiler.finishBlock();

    EXPECT_EQ(profOut.blockIndex, 1u);
    EXPECT_GE(profOut.totalTimeMs, 0.0);
    EXPECT_EQ(testSink->publishedCount, 1);
    EXPECT_EQ(testSink->lastIdentifier, "ProfilerOutput");
}

TEST(SourceTest, GpsSdrSimSourceFifo)
{
    GpsSdrSimSource simSource;
    SourceInput input{};
    snprintf(input.fifoPath, sizeof(input.fifoPath), "/tmp/gpsopencl/sim_data.fifo");
    bool initOk = simSource.initialize(input);
    EXPECT_TRUE(initOk);

    bool cmdOk = simSource.sendControlCommand("START");
    EXPECT_TRUE(cmdOk);
}
