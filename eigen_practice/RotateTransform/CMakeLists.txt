cmake_minimum_required(VERSION 2.8)
project(RotateTransform)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(/usr/include/eigen3)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)

set(SRC_CPP src/main.cpp
            src/RotateTransform.cpp)

add_executable(RotateTransform ${SRC_CPP})
