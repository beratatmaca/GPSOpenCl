# OpenCL context/device handling and FFT/mixing dispatch.
set(GPSOPENCL_GPU_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClGPUHandler.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClSpectrumEngine.cpp
    PARENT_SCOPE
)
