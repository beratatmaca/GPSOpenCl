# Telemetry output: abstract Sink plus file, ZMQ, NMEA, and JSON/console exporter implementations.
set(GPSOPENCL_SINK_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClFileSink.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClNmeaGenerator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClTelemetryExporter.cpp
)

# ZmqSink itself, and which factory implementation backs createZmqSinkIfEnabled(), are chosen here
# so no translation unit needs to #ifdef on GPSOPENCL_ENABLE_ZMQ.
if(GPSOPENCL_ENABLE_ZMQ)
    list(APPEND GPSOPENCL_SINK_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClZmqSink.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClZmqSinkFactoryEnabled.cpp
    )
else()
    list(APPEND GPSOPENCL_SINK_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClZmqSinkFactoryDisabled.cpp)
endif()

set(GPSOPENCL_SINK_SOURCES ${GPSOPENCL_SINK_SOURCES} PARENT_SCOPE)
