cmake_minimum_required(VERSION 2.8)
project(icp_demo)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(G2O REQUIRED)
find_package(CSparse REQUIRED)
include_directories(${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})
include_directories("/usr/local/include/eigen3")
include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(icp_demo ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(icp_demo g2o_core 
                                g2o_stuff 
                                g2o_csparse_extension  
                                g2o_types_slam3d
                                g2o_types_icp
                                ${CSPARSE_LIBRARY})