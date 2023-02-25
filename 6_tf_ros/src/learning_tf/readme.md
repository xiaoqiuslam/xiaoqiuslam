roscore

rosrun turtlesim turtlesim_node

cd catkin_ws_tf
source devel/setup.bash
catkin_ws_tf$rosrun learning_tf turtle_spawn

cd catkin_ws_tf
source devel/setup.bash
rosrun learning_tf turtle_pose_twist

cd catkin_ws_tf
source devel/setup.bash
rosrun learning_tf turtle_world_eye_tf

cd catkin_ws_tf
source devel/setup.bash
rosrun learning_tf turtle_point_follow



