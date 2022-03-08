add_subdirectory(Acquisition)
add_subdirectory(Tracking)
add_subdirectory(Settings)

add_executable(
    GPSOpenCl
    GPSOpenCl.cpp
)

target_include_directories(
    GPSOpenCl 
    PRIVATE 
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(
    GPSOpenCl
    PUBLIC
    Settings
)