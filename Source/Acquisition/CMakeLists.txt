add_library(
    Acquisition
)

#target_link_libraries(
#    Acquisition
#)

target_sources(
    Acquisition
    PRIVATE
    Acquisition.cpp
    Acquisition.h
)

target_include_directories(
    Acquisition 
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

