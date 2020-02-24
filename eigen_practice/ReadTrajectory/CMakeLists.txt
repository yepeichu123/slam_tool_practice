cmake_minimum_required(VERSION 2.8)
project(ReadTrajectory)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(/usr/include/eigen3)

set(SRC_CPP src/main.cpp
            src/ReadTrajectory.cpp
            src/CamPose.cpp)
add_executable(ReadTrajectory ${SRC_CPP})