CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

PROJECT( one_rgbd )

SET(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_COMPILER "g++")

FIND_PACKAGE( OpenCV REQUIRED )

FIND_PACKAGE( PCL REQUIRED )
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

ADD_EXECUTABLE(one_rgbd one_rgbd.cpp)
TARGET_LINK_LIBRARIES(one_rgbd ${OpenCV_LIBS} ${PCL_LIBRARIES} pcl_visualization)