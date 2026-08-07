#include "gtest/gtest.h"

#include "Common/GPSOpenClStructs.hpp"
#include "Sink/GPSOpenClZmqSink.hpp"

using namespace GPSOpenCl;

TEST(ZmqSinkTest, FailedBindDropsMessagesWithoutHangingOrCrashing)
{
    ZmqSink sink("bogus-transport://nowhere");
    SourceOutput srcOut{};
    sink.publishSourceOutput(srcOut);
}
