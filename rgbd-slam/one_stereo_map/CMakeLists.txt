CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

PROJECT( slam )

SET(CMAKE_CXX_COMPILER "g++")

SET(CMAKE_BUILD_TYPE Debug)


# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )
find_package(Pangolin REQUIRED)

add_executable(stereoVision stereoVision.cpp)
target_link_libraries(stereoVision ${OpenCV_LIBS} ${Pangolin_LIBRARIES})