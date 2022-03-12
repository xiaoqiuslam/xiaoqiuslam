cmake_minimum_required( VERSION 2.8 )
project( joinMap )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# opencv 
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# eigen 
include_directories("/usr/include/eigen3")

# pcl 
find_package(PCL REQUIRED COMPONENT common io)
include_directories( ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(plot_trajectory plot_trajectory.cpp)
target_link_libraries(plot_trajectory ${Pangolin_LIBRARIES})

add_executable(joinMap joinMap.cpp)
target_link_libraries( joinMap ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(joinMap_pnp joinMap_pnp.cpp)
target_link_libraries( joinMap_pnp ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(five_rgbd_map five_rgbd_map.cpp)
target_link_libraries(five_rgbd_map ${OpenCV_LIBS} ${PCL_LIBRARIES})