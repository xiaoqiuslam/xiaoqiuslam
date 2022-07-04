cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(1_NORMAL_ESTIMATION)

set(CMAKE_BUILD_TYPE Debug) # 设置Debug模式，否则无法在断点处停下

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(NormalEstimation NormalEstimation.cpp)
target_link_libraries(NormalEstimation ${PCL_LIBRARIES})