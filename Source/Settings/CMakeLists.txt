add_library(
    Settings
)

#target_link_libraries(
#    Acquisition
#)

target_sources(
    Settings
    PRIVATE
    Settings.cpp
    Settings.h
)

target_include_directories(
    Settings 
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

