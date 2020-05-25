cmake_minimum_required(VERSION 2.8)
project(test)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")

add_executable(test main.cpp)
