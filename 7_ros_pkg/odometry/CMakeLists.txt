cmake_minimum_required(VERSION 3.0.2)

project(odometry)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  turtlesim
)

find_package(Boost REQUIRED COMPONENTS system thread)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})

add_executable(ros_serial src/main.cc src/bsp_serial.cc)
target_link_libraries(ros_serial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

