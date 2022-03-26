add_executable(
    GPSOpenCl
    GPSOpenCl.cpp
    IOManagement/FileHandler.cpp
    Code/CACode.cpp
    Acquisition/Acquisition.cpp
    Utils/Utils.cpp
    Utils/FftUtils.cpp
    Logger/Logger.cpp
    Tracking/Tracking.cpp
    Navigation/Navigation.cpp
)

target_include_directories(
    GPSOpenCl 
    PRIVATE 
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(
    GPSOpenCl
    PUBLIC
    Qt5::Core
    OpenCL::OpenCL
    clFFT
)