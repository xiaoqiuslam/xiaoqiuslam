cmake_minimum_required(VERSION 2.8)

project(point_cloud )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_BUILD_TYPE Debug)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(PointIndices PointIndices.cpp)
target_link_libraries(PointIndices ${PCL_LIBRARIES})

add_executable(transformPointCloud transformPointCloud.cpp)
target_link_libraries(transformPointCloud ${PCL_LIBRARIES})