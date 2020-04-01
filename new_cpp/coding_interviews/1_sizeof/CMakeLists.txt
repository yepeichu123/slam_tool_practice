cmake_minimum_required(VERSION 2.8)
project(sizeof)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")

add_executable(sizeof main.cpp)