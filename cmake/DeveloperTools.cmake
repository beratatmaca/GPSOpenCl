# First-party C++ sources/headers; excludes Tools/gps-sdr-sim and fetched content.
file(GLOB GPSOPENCL_FORMAT_SOURCES
    ${CMAKE_SOURCE_DIR}/Source/*.cpp
    ${CMAKE_SOURCE_DIR}/Source/*.h
    ${CMAKE_SOURCE_DIR}/Tests/*.cpp
    ${CMAKE_SOURCE_DIR}/Tests/*.h
)

# clang-format: apply or verify project style on Source/ and Tests/.
find_program(CLANG_FORMAT_EXE clang-format)
if(CLANG_FORMAT_EXE)
    add_custom_target(format
        COMMAND ${CLANG_FORMAT_EXE} -i ${GPSOPENCL_FORMAT_SOURCES}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Formatting Source/ and Tests/ with clang-format"
        VERBATIM
    )
    add_custom_target(format-check
        COMMAND ${CLANG_FORMAT_EXE} --dry-run --Werror ${GPSOPENCL_FORMAT_SOURCES}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Checking Source/ and Tests/ formatting"
        VERBATIM
    )
else()
    message(STATUS "clang-format not found; 'format'/'format-check' targets unavailable")
endif()

# clang-tidy: lint Source/*.cpp against build/compile_commands.json.
find_program(CLANG_TIDY_EXE clang-tidy)
if(CLANG_TIDY_EXE)
    file(GLOB GPSOPENCL_TIDY_SOURCES ${CMAKE_SOURCE_DIR}/Source/*.cpp)
    add_custom_target(tidy
        COMMAND ${CLANG_TIDY_EXE} -p ${CMAKE_BINARY_DIR} ${GPSOPENCL_TIDY_SOURCES}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Running clang-tidy over Source/"
        VERBATIM
    )
else()
    message(STATUS "clang-tidy not found; 'tidy' target unavailable")
endif()

# Doxygen: generate API docs from the Doxygen comments in Source/ and Kernels/.
find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating Doxygen documentation into docs/html"
        VERBATIM
    )
else()
    message(STATUS "Doxygen not found; 'docs' target unavailable")
endif()
