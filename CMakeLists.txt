# @author Akash Patel
# @date Feb 18, 2019

cmake_minimum_required(VERSION 2.6)

project(utils)

find_package(DART 6.7.0 REQUIRED COMPONENTS utils-urdf)

# Set the top level directory for use in the code
add_definitions(-DTOP_LEVEL_PATH="${CMAKE_SOURCE_DIR}")

option(krang_sim_ach_SYSTEM_EIGEN "Use system-installed version of Eigen" OFF)

# Include and link to libraries

# Include Eigen
set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/eigen)

link_libraries(dart)

# Build the codebase library

# Include the project files
include_directories(include)
include_directories(${DART_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Collect the source files
file(GLOB src "src/*.cpp")
list(SORT src)

# Build the library
add_library(util SHARED ${src})

## Build scripts

# Install
install(TARGETS util DESTINATION /usr/local/lib/krang)
FILE(GLOB headers "include/krang/utils/*.hpp")
install(FILES ${headers} DESTINATION /usr/local/include/krang/util)

message(STATUS " ")

# uninstall target
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
