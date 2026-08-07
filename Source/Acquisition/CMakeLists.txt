# Doppler search, FFT correlation, and C/A code generation.
set(GPSOPENCL_ACQUISITION_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClAcquisition.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClCaCodeGenerator.cpp
    PARENT_SCOPE
)
