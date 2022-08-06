cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(show_pcd)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (show_two_pcd show_two_pcd.cpp)
target_link_libraries (show_two_pcd ${PCL_LIBRARIES} -lboost_thread )

add_executable (show_one_pcd show_one_pcd.cpp)
target_link_libraries (show_one_pcd ${PCL_LIBRARIES})

add_executable (b_show_one_pcd b_show_one_pcd.cpp)
target_link_libraries (b_show_one_pcd ${PCL_LIBRARIES})

add_executable (c_show_one_pcd c_show_one_pcd.cpp)
target_link_libraries (c_show_one_pcd ${PCL_LIBRARIES})

add_executable (d_show_one_pcd d_show_one_pcd.cpp)
target_link_libraries (d_show_one_pcd ${PCL_LIBRARIES})