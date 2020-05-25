cmake_minimum_required(VERSION 2.8)
project(KLTStereoMatching)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV 3.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(KLTStereoMatching main.cpp)
target_link_libraries(KLTStereoMatching ${OpenCV_LIBS})
