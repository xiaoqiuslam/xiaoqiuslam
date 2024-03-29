cmake_minimum_required(VERSION 2.8)

project(Rt)

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
set(CERES_LIBRARIES /usr/local/lib/)



find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(findEssentialMat findEssentialMat.cpp)
target_link_libraries(findEssentialMat ${OpenCV_LIBS})

add_executable( findFundamentalMat findFundamentalMat.cpp)

target_link_libraries( findFundamentalMat ${OpenCV_LIBS} )

add_executable( findHomography findHomography.cpp)

target_link_libraries( findHomography ${OpenCV_LIBS} )
