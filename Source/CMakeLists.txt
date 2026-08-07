add_subdirectory(Application)
add_subdirectory(Common)
add_subdirectory(Gpu)
add_subdirectory(Input)
add_subdirectory(Acquisition)
add_subdirectory(Tracking)
add_subdirectory(NavDecode)
add_subdirectory(Pvt)
add_subdirectory(Sink)

add_executable(
    GPSOpenCl
    Application/Main.cpp
    ${GPSOPENCL_APPLICATION_SOURCES}
    ${GPSOPENCL_COMMON_SOURCES}
    ${GPSOPENCL_GPU_SOURCES}
    ${GPSOPENCL_INPUT_SOURCES}
    ${GPSOPENCL_ACQUISITION_SOURCES}
    ${GPSOPENCL_TRACKING_SOURCES}
    ${GPSOPENCL_NAVDECODE_SOURCES}
    ${GPSOPENCL_PVT_SOURCES}
    ${GPSOPENCL_SINK_SOURCES}
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

# Copy kernels at build time so edits to .cl files reach the runtime search path on rebuild.
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/Kernels/Acquisition.cl
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_CURRENT_SOURCE_DIR}/../Kernels/Acquisition.cl
          ${CMAKE_CURRENT_BINARY_DIR}/Kernels/Acquisition.cl
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../Kernels/Acquisition.cl
)
add_custom_target(GPSOpenClKernels DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/Kernels/Acquisition.cl)
add_dependencies(GPSOpenCl GPSOpenClKernels)
