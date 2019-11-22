cmake_minimum_required(VERSION 3.13)
project(Files_class)

set(CMAKE_CXX_STANDARD 11)

add_executable(Files_class main.cpp FilesIO.h FilesIO.cpp Process.cpp Process.h ChangeIO.cpp ChangeIO.h)