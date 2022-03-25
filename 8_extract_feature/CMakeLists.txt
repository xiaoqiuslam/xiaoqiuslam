cmake_minimum_required(VERSION 2.8)

project(extract_feature)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

MESSAGE( OpenCV_INCLUDE_DIRS====: ${OpenCV_INCLUDE_DIRS})

include_directories(/home/q/xiaoqiuslamshizhanjiaocheng/extract_feature/ORBextractor.h)

add_executable(cv_orb_extract_feature cv_orb_extract_feature.cpp)
target_link_libraries(cv_orb_extract_feature ${OpenCV_LIBS})

MESSAGE( OpenCV_LIBS: ${OpenCV_LIBS})

add_executable(zhixin zhixin.cpp)
target_link_libraries(zhixin ${OpenCV_LIBS})

add_executable(hand_orb_extract_feature hand_orb_extract_feature.cpp)
target_link_libraries(hand_orb_extract_feature ${OpenCV_LIBS})

add_executable(orb_slam2_extract_feature orb_slam2_extract_feature.cpp ORBextractor.cc)
target_link_libraries(orb_slam2_extract_feature ${OpenCV_LIBS} )