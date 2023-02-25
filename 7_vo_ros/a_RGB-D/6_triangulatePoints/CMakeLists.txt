cmake_minimum_required( VERSION 2.8 )
project( vo1 )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV REQUIRED )
find_package( CSparse REQUIRED )

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${CSPARSE_INCLUDE_DIR}
    "/usr/include/eigen3/"
)


add_executable( triangulatePoints triangulatePoints.cpp)
target_link_libraries( triangulatePoints ${OpenCV_LIBS} )

add_executable( triangulatePoints_2 triangulatePoints_2.cpp)
target_link_libraries( triangulatePoints_2 ${OpenCV_LIBS} )