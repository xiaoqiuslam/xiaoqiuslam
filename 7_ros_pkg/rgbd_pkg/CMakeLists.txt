cmake_minimum_required(VERSION 3.0.2)

project(rgbd_pkg)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  tf
  geometry_msgs
  nav_msgs
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(RGBD_demo src/example.cpp)
target_link_libraries(RGBD_demo ${PCL_LIBRARIES}  ${OpenCV_LIBS} ${catkin_LIBRARIES} pcl_visualization)

# add_executable(ros_rgbd src/ros_rgbd.cpp)
# target_link_libraries(ros_rgbd ${PCL_LIBRARIES}  ${OpenCV_LIBS} ${catkin_LIBRARIES} pcl_visualization)












