#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <cmath>

int main(int argc, char** argv){

  ros::init(argc, argv, "my_tf_broadcaster");

  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  while (node.ok()){
    transform.setOrigin(tf::Vector3(1.0, 0.0, 0.0));

    Eigen::Matrix3d rotation_matrix3d_orb_rviz;
    rotation_matrix3d_orb_rviz << 0,  0, 1, 
                                 -1,  0, 0, 
                                  0, -1, 0;

    // 旋转矩阵转换为欧拉角,"2" represents the z axis , "0" x axis, "1" y axis
    Eigen::Vector3d euler_angle = rotation_matrix3d_orb_rviz.eulerAngles(0, 1, 2);
    std::cout << "绕z轴旋转的角度是 " << euler_angle.z() * 180 / M_PI << std::endl;

    // 绕z轴旋转的角度是 -90

    tf::Quaternion q;
    q.setRPY(0, 0, euler_angle.z());
    transform.setRotation( q );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    rate.sleep();
  }
  return 0;
};


