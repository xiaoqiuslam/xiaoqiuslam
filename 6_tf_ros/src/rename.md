[![小秋SLAM入门实战](/小秋SLAM实战教程.png)如果不能下载代码可以关注订阅号小秋SLAM实战教程添加微信slamshizhanjiaocheng获取代码](https://mp.weixin.qq.com/s/3Z129tEr6gWKgNAoXYYk4Q)

Sophus&ROS｜旋转矩阵和变换矩阵（从零开始手写VO视频课程）
[【从零开始学习SLAM】坐标变换 Eigen Sophus 旋转矩阵 轴角 旋转向量 欧拉角 四元数 位姿变换](https://chunqiushenye.blog.csdn.net/article/details/100080945)

## a_tf_pkg

    roscore

    catkin_make -DCATKIN_WHITELIST_PACKAGES="a_tf_pkg"
    source devel/setup.bash
    rosrun a_tf_pkg pub_tf

    rviz -d /media/q/q/xiaoqiuslam/6_tf/src/a_tf_pkg/rviz/pub_tf.rviz

    rostopic echo /tf

    rosrun rqt_tf_tree rqt_tf_tree 



上面代码发布的是静态坐标系之间的关系发布在tf话题上

