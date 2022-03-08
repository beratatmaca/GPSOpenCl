
#include(FetchContent)

#FetchContent_Declare(
#  clBLAS
#  GIT_REPOSITORY https://github.com/clMathLibraries/clBLAS.git
#  SOURCE_SUBDIR  src
#)

#FetchContent_MakeAvailable(clBLAS)

#include(clBLAS)