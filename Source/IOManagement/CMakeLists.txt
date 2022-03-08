add_library(
    IOManagement
)

target_sources(
    IOManagement
    PRIVATE
    FileHandler.cpp
    FileHandler.h
)

target_include_directories(
    IOManagement 
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_directories(
    IOManagement
    PUBLIC $<INSTALL_INTERFACE:${CMAKE_INSTALL_LIBDIR}>
)

