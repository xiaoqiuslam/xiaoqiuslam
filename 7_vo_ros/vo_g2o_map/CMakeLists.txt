cmake_minimum_required( VERSION 2.8 )
project ( myslam )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories(
        /usr/include/eigen3
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o-20170730_git/install/include
        ${PROJECT_SOURCE_DIR}
)

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
        /usr/local/lib/libSophus.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o-20170730_git/install/lib/libg2o_core.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o-20170730_git/install/lib/libg2o_stuff.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o-20170730_git/install/lib/libg2o_types_sba.so
)


include_directories( ${PROJECT_SOURCE_DIR}/include )

add_library( myslam SHARED
        src/frame.cpp
        src/mappoint.cpp
        src/map.cpp
        src/camera.cpp
        src/config.cpp
        src/g2o_types.cpp
        src/visual_odometry.cpp
        )

target_link_libraries( myslam
        ${THIRD_PARTY_LIBS}
        )

add_executable( run_vo test/run_vo.cpp )
target_link_libraries( run_vo myslam )
