#include "Sink/GPSOpenClZmqSinkFactory.hpp"

#include "Sink/GPSOpenClZmqSink.hpp"

namespace GPSOpenCl
{
std::shared_ptr<Sink> createZmqSinkIfEnabled()
{
    return std::make_shared<ZmqSink>();
}
}
