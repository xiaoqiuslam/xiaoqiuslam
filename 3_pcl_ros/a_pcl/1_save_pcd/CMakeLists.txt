cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_write)

find_package(PCL  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(a_save a_save.cpp)
target_link_libraries (a_save ${PCL_LIBRARIES})

add_executable(b_save b_save.cpp)
target_link_libraries (b_save ${PCL_LIBRARIES})