cmake_minimum_required(VERSION 2.8)
project(UndistorAndRemap)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenCV 3.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(UndistorAndRemap UndistorAndRemap.cpp)
target_link_libraries(UndistorAndRemap ${OpenCV_LIBS})