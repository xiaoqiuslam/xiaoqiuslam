roscore

rosrun turtlesim turtlesim_node


source devel/setup.bash
catkin_ws_tf$rosrun f_tf turtle_spawn


source devel/setup.bash
rosrun f_tf turtle_pose_twist


source devel/setup.bash
rosrun f_tf turtle_world_eye_tf


source devel/setup.bash
rosrun f_tf turtle_point_follow

[【从零开始学习SLAM】分解ros小海龟跟随代码](https://blog.csdn.net/qq_21950671/article/details/121949241)

