CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( slam )

SET(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "-O3")
SET( CMAKE_BUILD_TYPE DEBUG  )

FIND_PACKAGE( OpenCV REQUIRED )

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories("/usr/include/eigen3")

ADD_EXECUTABLE( pnp pnp.cpp)
target_link_libraries(pnp ${OpenCV_LIBS} )

ADD_EXECUTABLE( two_rgbd_map two_rgbd_map.cpp)
TARGET_LINK_LIBRARIES( two_rgbd_map ${OpenCV_LIBS}  ${PCL_LIBRARIES} )
