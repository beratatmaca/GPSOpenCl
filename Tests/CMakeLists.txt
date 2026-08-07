# Prefer a system GoogleTest so offline hosts can build; fetch only when absent.
find_package(GTest QUIET)
if(NOT GTest_FOUND)
  include(FetchContent)

  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
  )

  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)

  option(INSTALL_GTEST "Enable installation of googletest." OFF)

  FetchContent_MakeAvailable(googletest)
endif()

if(NOT TARGET GTest::gtest_main)
  add_library(GTest::gtest_main ALIAS gtest_main)
endif()

include(GoogleTest)

# Reuse the Source/ subdirectories' source lists for the test binary, out of the
# main GPSOpenCl target's binary tree so target names never collide.
add_subdirectory(${CMAKE_SOURCE_DIR}/Source/Application ${CMAKE_CURRENT_BINARY_DIR}/Source/Application)
add_subdirectory(${CMAKE_SOURCE_DIR}/Source/Common ${CMAKE_CURRENT_BINARY_DIR}/Source/Common)
add_subdirectory(${CMAKE_SOURCE_DIR}/Source/Gpu ${CMAKE_CURRENT_BINARY_DIR}/Source/Gpu)
add_subdirectory(${CMAKE_SOURCE_DIR}/Source/Input ${CMAKE_CURRENT_BINARY_DIR}/Source/Input)
add_subdirectory(${CMAKE_SOURCE_DIR}/Source/Acquisition ${CMAKE_CURRENT_BINARY_DIR}/Source/Acquisition)
add_subdirectory(${CMAKE_SOURCE_DIR}/Source/Tracking ${CMAKE_CURRENT_BINARY_DIR}/Source/Tracking)
add_subdirectory(${CMAKE_SOURCE_DIR}/Source/NavDecode ${CMAKE_CURRENT_BINARY_DIR}/Source/NavDecode)
add_subdirectory(${CMAKE_SOURCE_DIR}/Source/Pvt ${CMAKE_CURRENT_BINARY_DIR}/Source/Pvt)
add_subdirectory(${CMAKE_SOURCE_DIR}/Source/Sink ${CMAKE_CURRENT_BINARY_DIR}/Source/Sink)

add_executable(
    GPSOpenClTest
    Main.cpp
    ${GPSOPENCL_APPLICATION_SOURCES}
    ${GPSOPENCL_COMMON_SOURCES}
    ${GPSOPENCL_GPU_SOURCES}
    ${GPSOPENCL_INPUT_SOURCES}
    ${GPSOPENCL_ACQUISITION_SOURCES}
    ${GPSOPENCL_TRACKING_SOURCES}
    ${GPSOPENCL_NAVDECODE_SOURCES}
    ${GPSOPENCL_PVT_SOURCES}
    ${GPSOPENCL_SINK_SOURCES}
    GPUTest.cpp
    AcquisitionTest.cpp
    CaCodeTest.cpp
    SettingsTest.cpp
    TrackingTest.cpp
    NavigationDecoderTest.cpp
    ApplicationTest.cpp
    BoundedQueueTest.cpp
    TrackingWorkerPoolTest.cpp
    TelemetryJsonTest.cpp
    PVTSolverTest.cpp
    NmeaGeneratorTest.cpp
    AtmosphericCorrectionsTest.cpp
    StructsSourceSinkTest.cpp
    EndianTest.cpp
    LockDetectorTest.cpp
    ChannelTest.cpp
    GroundTruthVerificationTest.cpp
    SingleChannelGroundTruthTest.cpp
    DynamicMotionGroundTruthTest.cpp
    E2ETest.cpp
)

# ZmqSinkTest touches the real ZmqSink type, so it can only compile in when that type exists.
if(GPSOPENCL_ENABLE_ZMQ)
    target_sources(GPSOpenClTest PRIVATE ZmqSinkTest.cpp)
endif()

target_compile_definitions(GPSOpenClTest PUBLIC CTEST)

if(GPSOPENCL_ENABLE_ZMQ)
    target_link_libraries(GPSOpenClTest PRIVATE ${ZMQ_LIB})
endif()

target_include_directories(
  GPSOpenClTest
  PUBLIC 
  ${CMAKE_SOURCE_DIR}/Source
)

target_link_libraries(
  GPSOpenClTest
  PUBLIC 
  GTest::gtest_main
  OpenCL::OpenCL
)

gtest_discover_tests(
  GPSOpenClTest
  PROPERTIES
  LABELS "unit"
  DISCOVERY_TIMEOUT
  240
  DISCOVERY_PATTERN "*Test.cpp"
)

file(
  COPY 
  ${CMAKE_CURRENT_SOURCE_DIR}/Scripts/ConfigurationFile/DefaultConf.ini
  DESTINATION 
  ${CMAKE_CURRENT_BINARY_DIR}
)

file(
  COPY 
  ${CMAKE_CURRENT_SOURCE_DIR}/Scripts
  DESTINATION 
  ${CMAKE_CURRENT_BINARY_DIR}
)

# Copy kernels at build time so edits to .cl files reach the runtime search path on rebuild.
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/Kernels/Acquisition.cl
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_SOURCE_DIR}/Kernels/Acquisition.cl
          ${CMAKE_CURRENT_BINARY_DIR}/Kernels/Acquisition.cl
  DEPENDS ${CMAKE_SOURCE_DIR}/Kernels/Acquisition.cl
)
add_custom_target(GPSOpenClTestKernels DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/Kernels/Acquisition.cl)
add_dependencies(GPSOpenClTest GPSOpenClTestKernels)

file(
  COPY 
  ${CMAKE_SOURCE_DIR}/Tools
  DESTINATION 
  ${CMAKE_CURRENT_BINARY_DIR}
)