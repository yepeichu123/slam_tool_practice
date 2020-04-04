cmake_minimum_required(VERSION 2.8)
project(circle_fit)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(G2O REQUIRED)
find_package(CSparse REQUIRED)
include_directories(${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories("/usr/local/include/eigen3/")

set(SRC_CPP ${PROJECT_SOURCE_DIR}/src/main.cpp
            ${PROJECT_SOURCE_DIR}/src/circle_graph.cpp)

add_executable(circle_fit ${SRC_CPP})
target_link_libraries(circle_fit g2o_core 
                                 g2o_stuff 
                                 g2o_solver_csparse
                                 g2o_csparse_extension 
                                 ${CSPARSE_LIBRARY})
