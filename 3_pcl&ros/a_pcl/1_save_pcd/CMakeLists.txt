cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_write)

find_package(PCL  REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(1_save_pcd 1_save_pcd.cpp)
target_link_libraries (1_save_pcd ${PCL_LIBRARIES})

add_executable(2_save_pcd 2_save_pcd.cpp)
target_link_libraries (2_save_pcd ${PCL_LIBRARIES})