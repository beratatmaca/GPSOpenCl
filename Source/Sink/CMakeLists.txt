# Telemetry output: abstract Sink plus file, ZMQ, NMEA, and JSON/console exporter implementations.
set(GPSOPENCL_SINK_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClFileSink.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClZmqSink.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClNmeaGenerator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClTelemetryExporter.cpp
    PARENT_SCOPE
)
