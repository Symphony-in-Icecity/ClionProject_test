cmake_minimum_required(VERSION 3.13)
project(Eigen_test)
#uncomment the following 4 lines to use the Eigen library

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

set(CMAKE_CXX_STANDARD 11)

add_executable(Eigen_test main.cpp)