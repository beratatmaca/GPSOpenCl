#ifndef INCLUDED_GPSOPENCL_ZMQSINKFACTORY_HPP
#define INCLUDED_GPSOPENCL_ZMQSINKFACTORY_HPP

/** @file GPSOpenClZmqSinkFactory.hpp
 *  @brief Build-time selectable ZMQ sink construction. Exactly one of
 *   GPSOpenClZmqSinkFactoryEnabled.cpp or GPSOpenClZmqSinkFactoryDisabled.cpp
 *   is compiled in, chosen by the GPSOPENCL_ENABLE_ZMQ CMake option, so callers
 *   never branch on a feature macro.
 */

#include "Sink/GPSOpenClSink.hpp"

#include <memory>

namespace GPSOpenCl
{
/** @brief Construct a ZmqSink if ZMQ support was compiled in.
 *  @return New ZmqSink, or nullptr if GPSOPENCL_ENABLE_ZMQ is off. */
std::shared_ptr<Sink> createZmqSinkIfEnabled();
}

#endif
