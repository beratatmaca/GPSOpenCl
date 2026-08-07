# Shared types, config parsing, and profiling; no OpenCL/ZMQ dependency.
set(GPSOPENCL_COMMON_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClProfiler.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClSettings.cpp
    PARENT_SCOPE
)
