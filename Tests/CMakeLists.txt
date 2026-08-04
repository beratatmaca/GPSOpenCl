include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)

option(INSTALL_GTEST "Enable installation of googletest." OFF)

FetchContent_MakeAvailable(googletest)

include(GoogleTest)

add_executable(
    GPSOpenClTest
    Main.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClGPUHandler.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClGPUCompute.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClCode.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClSettings.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClAcquisition.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClTracking.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClLockDetector.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClChannel.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClApplication.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClNavigationDecoder.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClPVTSolver.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClNmeaGenerator.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClAtmosphericCorrections.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClFileSource.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClGpsSdrSimSource.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClFileSink.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClZmqSink.cpp
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClProfiler.cpp
    GPUTest.cpp
    CaCodeTest.cpp
    SettingsTest.cpp
    TrackingTest.cpp
    NavigationDecoderTest.cpp
    PVTSolverTest.cpp
    NmeaGeneratorTest.cpp
    AtmosphericCorrectionsTest.cpp
    StructsSourceSinkTest.cpp
    LockDetectorTest.cpp
    ChannelTest.cpp
    GroundTruthVerificationTest.cpp
    SingleChannelGroundTruthTest.cpp
    DynamicMotionGroundTruthTest.cpp
    E2ETest.cpp
)

target_compile_definitions(GPSOpenClTest PUBLIC CTEST)

if(GPSOPENCL_ENABLE_ZMQ)
    target_compile_definitions(GPSOpenClTest PRIVATE GPSOPENCL_ENABLE_ZMQ)
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
  gtest_main
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