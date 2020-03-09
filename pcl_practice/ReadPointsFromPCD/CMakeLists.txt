cmake_minimum_required(VERSION 2.8)
project(ReadPointsFromPCD)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(PCL 1.6.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ReadPointsFromPCD ${PROJECT_SOURCE_DIR}/src/ReadPointsFromPCD.cpp)
target_link_libraries(ReadPointsFromPCD ${PCL_LIBRARIES})