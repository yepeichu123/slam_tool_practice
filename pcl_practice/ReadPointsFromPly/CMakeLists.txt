cmake_minimum_required(VERSION 2.8)
project(ReadPointsFromPly)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wno-deprecated")
set(CMAKE_BUILD_TYPE "Release")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_libraries(${PCL_LIBRARY_DIRS})

set(SRC_CPP ${PROJECT_SOURCE_DIR}/src/ReadPointsFromPly.cpp)

add_executable(ReadPOintsFromPly ${SRC_CPP})
target_link_libraries(ReadPOintsFromPly ${PCL_LIBRARIES})