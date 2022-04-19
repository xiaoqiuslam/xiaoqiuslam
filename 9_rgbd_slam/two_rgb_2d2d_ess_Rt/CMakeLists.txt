cmake_minimum_required(VERSION 2.8)

project(vo1)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -msse4")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# find_package(Eigen3 REQUIRED)
set(Eigen3_DIR /usr/lib/cmake/eigen3/Eigen3Config.cmake)
set(EIGEN3_INCLUDE_DIRS /usr/local/include/eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

# find_package(G2O REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(G2O_INCLUDE_DIRS /usr/local/include/g2o/)
set(CERES_LIBRARIES /usr/local/lib/)
include_directories(${G2O_INCLUDE_DIRS})

find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(two_rgb_rt_2d2d two_rgb_rt_2d2d.cpp)
target_link_libraries(two_rgb_rt_2d2d ${OpenCV_LIBS})



