add_library(
    Tracking
)

#target_link_libraries(
#    Tracking
#)

target_sources(
    Tracking
    PRIVATE
    Tracking.cpp
    Tracking.h
)

target_include_directories(
    Tracking 
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
