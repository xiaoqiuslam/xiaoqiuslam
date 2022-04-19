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
add_executable(creat_point_cloud_pub src/creat_point_cloud_pub.cpp)
target_link_libraries(creat_point_cloud_pub ${PCL_LIBRARIES}  ${catkin_LIBRARIES} pcl_visualization)

add_executable(point_cloud_transform src/point_cloud_transform.cpp)
target_link_libraries(point_cloud_transform ${PCL_LIBRARIES}  ${catkin_LIBRARIES} pcl_visualization )

add_executable(creat_spiral_point_cloud src/creat_spiral_point_cloud.cpp)
target_link_libraries(creat_spiral_point_cloud ${PCL_LIBRARIES}  ${catkin_LIBRARIES}pcl_visualization )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(creat_image_point_cloud src/creat_image_point_cloud.cpp)
target_link_libraries(creat_image_point_cloud ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} pcl_visualization )



add_executable(laser_scan src/laser_scan.cpp)
target_link_libraries(laser_scan ${catkin_LIBRARIES} pcl_visualization )


add_executable(ros_point_cloud src/ros_point_cloud.cpp)
target_link_libraries(ros_point_cloud ${catkin_LIBRARIES} pcl_visualization)

