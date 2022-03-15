cmake_minimum_required(VERSION 3.14)

project(GPSOpenCl
        VERSION 0.1
        LANGUAGES CXX
        )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(OpenCL REQUIRED)
find_package(clFFT REQUIRED)

add_subdirectory(Source)
add_subdirectory(Test)

enable_testing()
