# PLL/DLL tracking loops, lock detection, per-satellite channel state, and the worker pool.
set(GPSOPENCL_TRACKING_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClTracking.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClChannel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClLockDetector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClTrackingWorkerPool.cpp
    PARENT_SCOPE
)
