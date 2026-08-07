cmake_minimum_required(VERSION 3.14)

project(GPSOpenCl
        VERSION 0.1
        LANGUAGES CXX C
        )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g -D_GLIBCXX_ASSERTIONS")
option(GPSOPENCL_STDLIB_DEBUG "Enable _GLIBCXX_DEBUG checked containers in Debug builds" OFF)
if(GPSOPENCL_STDLIB_DEBUG)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_GLIBCXX_DEBUG")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-fno-math-errno -fno-trapping-math -fcx-limited-range)
option(GPSOPENCL_NATIVE "Compile with -march=native for the build machine" OFF)
if(GPSOPENCL_NATIVE)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-march=native" GPSOPENCL_HAS_MARCH_NATIVE)
    if(GPSOPENCL_HAS_MARCH_NATIVE)
        add_compile_options(-march=native)
    endif()
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    include(CheckIPOSupported)
    check_ipo_supported(RESULT GPSOPENCL_IPO_SUPPORTED OUTPUT GPSOPENCL_IPO_MESSAGE)
    if(GPSOPENCL_IPO_SUPPORTED)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
    else()
        message(STATUS "IPO/LTO not supported: ${GPSOPENCL_IPO_MESSAGE}")
    endif()
endif()


find_package(OpenCL QUIET)

if(NOT OpenCL_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        opencl_headers
        GIT_REPOSITORY https://github.com/KhronosGroup/OpenCL-Headers.git
        GIT_TAG v2023.12.14
    )
    FetchContent_Declare(
        opencl_icd_loader
        GIT_REPOSITORY https://github.com/KhronosGroup/OpenCL-ICD-Loader.git
        GIT_TAG v2023.12.14
    )
    set(OPENCL_ICD_LOADER_BUILD_TESTING OFF CACHE BOOL "" FORCE)
    set(GPSOPENCL_SAVED_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
    set(BUILD_SHARED_LIBS OFF)
    FetchContent_MakeAvailable(opencl_headers opencl_icd_loader)
    set(BUILD_SHARED_LIBS ${GPSOPENCL_SAVED_BUILD_SHARED_LIBS})

    if(NOT TARGET OpenCL::OpenCL)
        add_library(OpenCL::OpenCL ALIAS OpenCL)
    endif()
endif()

option(GPSOPENCL_ENABLE_ZMQ "Enable ZeroMQ telemetry publishing" ON)
if(GPSOPENCL_ENABLE_ZMQ)
    find_library(ZMQ_LIB zmq)
    if(ZMQ_LIB)
        message(STATUS "ZeroMQ library found: ${ZMQ_LIB}")
    else()
        message(STATUS "ZeroMQ library not found; disabling ZMQ feature")
        set(GPSOPENCL_ENABLE_ZMQ OFF CACHE BOOL "" FORCE)
    endif()
endif()

add_subdirectory(Source)

option(GPSOPENCL_BUILD_TESTS "Build the GoogleTest suite" ON)
if(GPSOPENCL_BUILD_TESTS)
    enable_testing()
    add_subdirectory(Tests)
endif()

if(EXISTS ${CMAKE_SOURCE_DIR}/Tools/gps-sdr-sim/CMakeLists.txt)
    add_subdirectory(Tools/gps-sdr-sim)
else()
    message(STATUS "Tools/gps-sdr-sim not present (run: git submodule update --init); simulator will not be built")
endif()

include(cmake/DeveloperTools.cmake)