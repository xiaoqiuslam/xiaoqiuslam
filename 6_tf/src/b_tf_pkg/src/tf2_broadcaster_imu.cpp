#include <cstdio>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <tf_conversions/tf_eigen.h>

int main(int argc, char **argv){

    Eigen::Matrix3d R_bc;
    Eigen::Vector3d t_bc;
    Eigen::Matrix3d R;
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

  geometry_msgs::TransformStamped geometry_msgs_transform_stamped;
  geometry_msgs_transform_stamped.header.frame_id = "map";
  geometry_msgs_transform_stamped.child_frame_id = "base_link";
  geometry_msgs_transform_stamped.transform.translation.x = tf_trans.x();
  geometry_msgs_transform_stamped.transform.translation.y = tf_trans.y();
  geometry_msgs_transform_stamped.transform.translation.z = tf_trans.z();
  geometry_msgs_transform_stamped.transform.rotation.x = tf_quat.x();
  geometry_msgs_transform_stamped.transform.rotation.y = tf_quat.y();
  geometry_msgs_transform_stamped.transform.rotation.z = tf_quat.z();
  geometry_msgs_transform_stamped.transform.rotation.w = tf_quat.w();

  ros::init(argc,argv, "my_tf2_broadcaster_imu");
  ros::NodeHandle node;
  ros::Time::init();
  ros::Rate rate(1.0);
  static tf2_ros::StaticTransformBroadcaster static_transform_broadcaster;
  tf2_ros::TransformBroadcaster tfb;
  while (node.ok()){
    geometry_msgs_transform_stamped.header.stamp = ros::Time::now();
    // 发布的话题是　/tf_static
    static_transform_broadcaster.sendTransform(geometry_msgs_transform_stamped);
    rate.sleep();
  }
  ros::spin();
  return 0;
};
