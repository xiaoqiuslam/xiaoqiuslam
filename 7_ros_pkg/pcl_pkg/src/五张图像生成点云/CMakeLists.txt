cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")
# 添加c++ 11标准支持
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

# Eigen
include_directories("/usr/local/include/eigen3")

# 寻找OpenCV库
find_package(OpenCV REQUIRED)
# 添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})


find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)

add_executable(joinMap_pangolin joinMap_pangolin.cpp)
target_link_libraries(joinMap_pangolin ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

add_executable( joinMap_pcd joinMap_pcd.cpp)
target_link_libraries( joinMap_pcd ${OpenCV_LIBS} ${PCL_LIBRARIES} )
