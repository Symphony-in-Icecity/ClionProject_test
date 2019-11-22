cmake_minimum_required(VERSION 3.13)
project(class_test)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
set(CMAKE_CXX_STANDARD 11)

add_executable(class_test main.cpp Spline.cpp Spline.h RangeImpl.h QuinticPolynomial.cpp QuinticPolynomial.h QuarticPolynomial.cpp QuarticPolynomial.h)