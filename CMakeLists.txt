cmake_minimum_required(VERSION 3.14)

project(GPSOpenCl
        VERSION 0.1
        LANGUAGES CXX C
        )

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_GLIBCXX_DEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DCMAKE_EXPORT_COMPILE_COMMANDS=1")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")


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
    FetchContent_MakeAvailable(opencl_headers opencl_icd_loader)

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

enable_testing()

add_subdirectory(Source)
add_subdirectory(Tests)
add_subdirectory(Tools/gps-sdr-sim)