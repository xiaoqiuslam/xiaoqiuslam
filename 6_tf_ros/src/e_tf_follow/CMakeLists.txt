cmake_minimum_required(VERSION 3.0.2)
project(e_tf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  turtlesim
  geometry_msgs
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})

add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})

# roscore
# rosrun turtlesim turtlesim_node

# 发布速度消息
add_executable(turtle_pose_twist src/turtle_pose_twist.cpp)
target_link_libraries(turtle_pose_twist ${catkin_LIBRARIES})

# 创造第二只海龟
add_executable(turtle_spawn src/turtle_spawn.cpp)
target_link_libraries(turtle_spawn ${catkin_LIBRARIES})


# １、订阅小海龟的实时位置姿态
# ２、解析小海龟的实时位置姿态
# ３、发布小海龟相对于世界坐标系之间的tf
# ４、发布相机相对于小海龟坐标系之间的tf
# ５、将相机坐标系下面的点通过tf变换到小海龟１坐标系下、变换到小海龟２坐标系下、变换到世界坐标系下
add_executable(turtle_world_eye_tf src/turtle_world_eye_tf.cpp)
target_link_libraries(turtle_world_eye_tf ${catkin_LIBRARIES})


# add_executable(turtle_transform_point src/turtle_transform_point.cpp)
# target_link_libraries(turtle_transform_point ${catkin_LIBRARIES})

add_executable(turtle_point_follow src/turtle_point_follow.cpp)
target_link_libraries(turtle_point_follow ${catkin_LIBRARIES})


# 发布静态坐标系
# add_executable(turtle_eye_static_tf_broadcaster src/turtle_eye_static_tf_broadcaster.cpp)
# target_link_libraries(turtle_eye_static_tf_broadcaster ${catkin_LIBRARIES})

# 订阅静态坐标系，将眼睛下的点转换到海龟坐标系下面
# add_executable(turtle_eye_static_tf_listener src/turtle_eye_static_tf_listener.cpp)
# target_link_libraries(turtle_eye_static_tf_listener ${catkin_LIBRARIES})

# 发布动态坐标系
# add_executable(turtle_world_dynamic_tf_broadcaster src/turtle_world_dynamic_tf_broadcaster.cpp)
# target_link_libraries(turtle_world_dynamic_tf_broadcaster ${catkin_LIBRARIES})