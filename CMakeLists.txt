cmake_minimum_required(VERSION 3.0)
project(psopt_sandbox)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

set(PSOPT_BUILD_EXAMPLES 1)
add_subdirectory(lib/psopt-cmake)

include_directories(include
    lib/psopt-cmake/lusol/csrc
    lib/psopt-cmake/dmatrix/include
    lib/psopt-cmake/PSOPT/src
    ${CXSPARSE_INCLUDE_DIRS}
    ${IPOPT_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

add_executable(psopt_sandbox
    src/move_box.cpp
    )
target_link_libraries(psopt_sandbox
    psopt
    adolc
    dmatrix
    lusol
    gtest
    gtest_main
    ${GTEST_LIBRARIES}
    pthread)
