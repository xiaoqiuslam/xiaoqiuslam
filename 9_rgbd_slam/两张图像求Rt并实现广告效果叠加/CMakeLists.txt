
cmake_minimum_required( VERSION 2.8 )
project( homography )

set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( homography homography.cpp)

target_link_libraries( homography ${OpenCV_LIBS} )