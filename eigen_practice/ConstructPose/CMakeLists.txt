cmake_minimum_required(VERSION 2.8)
project(ConstructPose)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(/usr/include/eigen3)

set(SRC_CPP ConstructPose.cpp)
add_executable(ConstructPose ${SRC_CPP})