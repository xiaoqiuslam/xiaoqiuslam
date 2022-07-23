cmake_minimum_required(VERSION 2.8)

project(eigen_sophus_tutorial)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

include_directories("/usr/include/eigen3")

# 安装视觉SLAM十四讲第一版slambook下面的Sophus
set(Sophus_LIBRARIES /usr/local/lib/libSophus.so)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable( 1_eigen_vector_matrix 1_eigen_vector_matrix.cpp)
add_executable( 2_eigen_math 2_eigen_math.cpp)
add_executable( 3_eigen_pose_transform 3_eigen_pose_transform.cpp)

add_executable(4_plot_trajectory 4_plot_trajectory.cpp)
target_link_libraries(4_plot_trajectory ${Pangolin_LIBRARIES})

add_executable( 5_trajectory_transform 5_trajectory_transform.cpp)
target_link_libraries( 5_trajectory_transform ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( 6_sophus_pose_transform 6_sophus_pose_transform.cpp)
target_link_libraries( 6_sophus_pose_transform ${Sophus_LIBRARIES} )

add_executable(7_trajectory_error 7_trajectory_error.cpp)
target_link_libraries(7_trajectory_error ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})

add_executable(8_coordinateTransform 8_coordinateTransform.cpp)





