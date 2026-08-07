file(GLOB_RECURSE GPSOPENCL_FORMAT_SOURCES
    ${CMAKE_SOURCE_DIR}/Source/*.cpp
    ${CMAKE_SOURCE_DIR}/Source/*.hpp
    ${CMAKE_SOURCE_DIR}/Tests/*.cpp
    ${CMAKE_SOURCE_DIR}/Tests/*.hpp
)

find_program(CLANG_FORMAT_EXE NAMES clang-format-20 clang-format-19 clang-format-18 clang-format)
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

find_program(CLANG_TIDY_EXE NAMES clang-tidy-20 clang-tidy-19 clang-tidy-18 clang-tidy)
if(CLANG_TIDY_EXE)
    file(GLOB_RECURSE GPSOPENCL_TIDY_SOURCES ${CMAKE_SOURCE_DIR}/Source/*.cpp)
    add_custom_target(tidy
        COMMENT "Running clang-tidy over Source/"
    )
    foreach(GPSOPENCL_TIDY_FILE ${GPSOPENCL_TIDY_SOURCES})
        add_custom_command(
            TARGET tidy
            POST_BUILD
            COMMAND ${CLANG_TIDY_EXE} -p ${CMAKE_BINARY_DIR} ${GPSOPENCL_TIDY_FILE}
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            VERBATIM
        )
    endforeach()
else()
    message(STATUS "clang-tidy not found; 'tidy' target unavailable")
endif()

find_program(CPPCHECK_EXE NAMES cppcheck)
if(CPPCHECK_EXE)
    add_custom_target(cppcheck
        COMMAND ${CPPCHECK_EXE}
                --project=${CMAKE_BINARY_DIR}/compile_commands.json
                --file-filter=${CMAKE_SOURCE_DIR}/Source/*
                --enable=warning,performance,portability
                --inline-suppr
                --error-exitcode=1
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Running cppcheck over Source/"
        VERBATIM
    )
else()
    message(STATUS "cppcheck not found; 'cppcheck' target unavailable")
endif()

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
