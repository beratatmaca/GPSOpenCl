cmake_minimum_required(VERSION 3.14)

project(GPSOpenCl
        VERSION 0.1
        LANGUAGES CXX
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


find_package(OpenCL REQUIRED)

add_subdirectory(Source)
add_subdirectory(Tests)

enable_testing()