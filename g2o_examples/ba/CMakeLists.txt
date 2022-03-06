cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")
# 添加c++ 11标准支持
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

# Eigen
include_directories("/usr/include/eigen3")


add_executable(ba_demo
  ba_demo.cpp
)

include_directories(${CHOLMOD_INCLUDE_DIR})

target_link_libraries(ba_demo core solver_cholmod types_sba)


