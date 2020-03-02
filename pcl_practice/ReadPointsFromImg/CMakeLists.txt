cmake_minimum_required(VERSION 2.8)
project(ReadPointsFromImg)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenCV 3.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ReadPointsFromImg src/ReadPointsFromImg.cpp)
target_link_libraries(ReadPointsFromImg ${OpenCV_LIBS} ${PCL_LIBRARIES})