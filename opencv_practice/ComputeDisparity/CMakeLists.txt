cmake_minimum_required(VERSION 2.8)
project(ComputeDisparity)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenCV 3.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ComputeDisparity main.cpp)
target_link_libraries(ComputeDisparity ${OpenCV_LIBS})