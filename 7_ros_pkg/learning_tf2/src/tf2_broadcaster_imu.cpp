#include <cstdio>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <tf_conversions/tf_eigen.h>

int main(int argc, char **argv){

    ros::init(argc,argv, "my_tf2_broadcaster_imu");

    ros::NodeHandle node;

    // 外参数
    Eigen::Matrix3d R_bc;   // cam to body
    Eigen::Vector3d t_bc;     // cam to body
    Eigen::Matrix3d R;   // 把body坐标系朝向旋转一下,得到相机坐标系，好让它看到landmark,  相机坐标系的轴在body坐标系中的表示
    // 相机朝着轨迹里面看， 特征点在轨迹外部， 这里我们采用这个
    R << 0, 0, -1,
            -1, 0,  0,
            0, 1,  0;
    R_bc = R;
    t_bc = Eigen::Vector3d(0.05,0.04,0.03);


    Eigen::Quaterniond eigen_quat(R_bc);
    tf::Quaternion tf_quat;
    tf::quaternionEigenToTF(eigen_quat, tf_quat);
    tf::Vector3 tf_trans;
    tf::vectorEigenToTF(t_bc, tf_trans);

  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.frame_id = "imu";
  static_transformStamped.child_frame_id = "camera";
  static_transformStamped.transform.translation.x = tf_trans.x();
  static_transformStamped.transform.translation.y = tf_trans.y();
  static_transformStamped.transform.translation.z = tf_trans.z();

  static_transformStamped.transform.rotation.x = tf_quat.x();
  static_transformStamped.transform.rotation.y = tf_quat.y();
  static_transformStamped.transform.rotation.z = tf_quat.z();
  static_transformStamped.transform.rotation.w = tf_quat.w();


  ros::Time::init();
  ros::Rate rate(1.0);
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  tf2_ros::TransformBroadcaster tfb;
  while (node.ok()){
    static_transformStamped.header.stamp = ros::Time::now();
    // 发布的话题是　/tf_static
    static_broadcaster.sendTransform(static_transformStamped);

    rate.sleep();
  }
  ros::spin();
  return 0;
};
