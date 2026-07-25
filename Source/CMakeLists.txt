add_executable(
    GPSOpenCl
    Main.cpp
    GPSOpenClGPUHandler.cpp
    GPSOpenClGPUCompute.cpp
    GPSOpenClCode.cpp
    GPSOpenClSettings.cpp
    GPSOpenClChannel.cpp
    GPSOpenClAcquisition.cpp
    GPSOpenClTracking.cpp
    GPSOpenClApplication.cpp
    GPSOpenClNavigationDecoder.cpp
    GPSOpenClPVTSolver.cpp
    GPSOpenClNmeaGenerator.cpp
    GPSOpenClAtmosphericCorrections.cpp
    GPSOpenClFileSource.cpp
    GPSOpenClGpsSdrSimSource.cpp
    GPSOpenClFileSink.cpp
    GPSOpenClZmqSink.cpp
    GPSOpenClProfiler.cpp
)

target_include_directories(
    GPSOpenCl 
    PRIVATE 
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/Source
)

target_link_libraries(
    GPSOpenCl
    PUBLIC
    OpenCL::OpenCL
)

target_compile_definitions(GPSOpenCl PRIVATE CL_TARGET_OPENCL_VERSION=220)

if(GPSOPENCL_ENABLE_ZMQ)
    target_compile_definitions(GPSOpenCl PRIVATE GPSOPENCL_ENABLE_ZMQ)
    target_link_libraries(GPSOpenCl PRIVATE ${ZMQ_LIB})
endif()

file(
  COPY 
  ${CMAKE_CURRENT_SOURCE_DIR}/../Tests/Scripts/ConfigurationFile/DefaultConf.ini
  DESTINATION 
  ${CMAKE_CURRENT_BINARY_DIR}
)

file(
  COPY 
  ${CMAKE_CURRENT_SOURCE_DIR}/../Kernels
  DESTINATION 
  ${CMAKE_CURRENT_BINARY_DIR}
)