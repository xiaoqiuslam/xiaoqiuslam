cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "-std=c++11 -O2")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(joinMap_pangolin joinMap_pangolin.cpp)
target_link_libraries(joinMap_pangolin ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

add_executable( joinMap_pcd joinMap_pcd.cpp)
target_link_libraries( joinMap_pcd ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable(pangolin_plot_trajectory pangolin_plot_trajectory.cpp)
target_link_libraries(pangolin_plot_trajectory ${Pangolin_LIBRARIES})
