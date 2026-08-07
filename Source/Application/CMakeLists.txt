# Top-level pipeline wiring; depends on every other Source/ module.
set(GPSOPENCL_APPLICATION_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClApplication.cpp
    PARENT_SCOPE
)
