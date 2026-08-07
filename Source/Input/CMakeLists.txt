# IQ sample sources: file playback and gps-sdr-sim FIFO.
set(GPSOPENCL_INPUT_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClFileSource.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClGpsSdrSimSource.cpp
    PARENT_SCOPE
)
