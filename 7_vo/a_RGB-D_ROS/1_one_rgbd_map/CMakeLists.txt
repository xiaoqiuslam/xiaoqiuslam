cmake_minimum_required(VERSION 3.0.2)

project(pcl_pkg)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(creat_image_point_cloud creat_image_point_cloud.cpp)
target_link_libraries(creat_image_point_cloud ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} pcl_visualization )
