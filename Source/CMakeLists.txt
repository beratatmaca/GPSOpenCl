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
)

target_include_directories(
    GPSOpenCl 
    PRIVATE 
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(
    GPSOpenCl
    PUBLIC
    OpenCL::OpenCL
)

target_compile_definitions(GPSOpenCl PRIVATE CL_TARGET_OPENCL_VERSION=220)


file(
  COPY 
  ${CMAKE_CURRENT_SOURCE_DIR}/../Tests/Scripts/ConfigurationFile/DefaultConf.ini
  DESTINATION 
  ${CMAKE_CURRENT_BINARY_DIR}
)