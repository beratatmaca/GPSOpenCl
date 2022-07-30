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
    ${CMAKE_SOURCE_DIR}/Source/GPSOpenClChannel.cpp
    GPUTest.cpp
    CaCodeTest.cpp
    SettingsTest.cpp
    AcquisitionTest.cpp
    #TrackingTest.cpp
)

target_compile_definitions(GPSOpenClTest PUBLIC CTEST)

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