cmake_minimum_required(VERSION 3.14)

project(GPSOpenCl
        VERSION 0.1
        LANGUAGES CXX
        )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

add_subdirectory(External)
add_subdirectory(Source)
add_subdirectory(Test)
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/External" )

enable_testing()
