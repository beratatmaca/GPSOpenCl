#include "gtest/gtest.h"

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
    EXPECT_EQ(acqOut.structVersion, STRUCT_VERSION_1);

    TrackingOutput trackOut{};
    EXPECT_EQ(trackOut.structVersion, STRUCT_VERSION_1);
    EXPECT_EQ(trackOut.channelState, 0u);
    EXPECT_EQ(trackOut.carrierLockIndicator, 0.0);
    EXPECT_EQ(trackOut.codeLockRatio, 0.0);

    NavDecoderOutput navOut{};
    EXPECT_EQ(navOut.structVersion, STRUCT_VERSION_1);

    PvtSolverOutput pvtOut{};
    EXPECT_EQ(pvtOut.structVersion, STRUCT_VERSION_1);

    AtmosphericOutput atmOut{};
    EXPECT_EQ(atmOut.structVersion, STRUCT_VERSION_1);

    NmeaGeneratorOutput nmeaOut{};
    EXPECT_EQ(nmeaOut.structVersion, STRUCT_VERSION_1);

    ProfilerOutput profOut{};
    EXPECT_EQ(profOut.structVersion, STRUCT_VERSION_1);
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
