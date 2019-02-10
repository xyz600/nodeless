cmake_minimum_required (VERSION 3.0)
project(xyz_nodeless)

set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O2")

add_executable(solver main.cpp)