# 开发环境
# Ubuntu 16.04、ros Kinetic 默认安装的 opnecv3.3.1
cmake_minimum_required(VERSION 3.0.2)

project(image_pkg)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

add_executable(pub_image_topic src/pub_image_topic.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(pub_image_topic ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(pub_video2image_topic src/pub_video2image_topic.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(pub_video2image_topic ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(pub_camera2image_topic src/pub_camera2image_topic.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(pub_camera2image_topic ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(sub_image_topic src/sub_image_topic.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(sub_image_topic ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(sub_compress_image_topic src/sub_compress_image_topic.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(sub_compress_image_topic ${OpenCV_LIBS} ${catkin_LIBRARIES})