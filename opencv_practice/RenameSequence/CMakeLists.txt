cmake_minimum_required(VERSION 2.8)
project(RenameSequence)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenCV 3.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)

set(SRC_CPP src/main.cpp
            src/RenameSequence.cpp)

add_executable(RenameSequence ${SRC_CPP})
target_link_libraries(RenameSequence ${OpenCV_LIBS})