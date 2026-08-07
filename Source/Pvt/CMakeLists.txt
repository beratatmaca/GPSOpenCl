# Measurement assembly, atmospheric corrections, and the WLS position/velocity/time solver.
set(GPSOPENCL_PVT_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClMeasurementAssembler.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClAtmosphericCorrections.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GPSOpenClPVTSolver.cpp
    PARENT_SCOPE
)
