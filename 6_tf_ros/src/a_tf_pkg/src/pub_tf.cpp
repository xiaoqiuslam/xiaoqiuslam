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

  Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d ( 0, 0, 1 ) );
  std::cout << "angle is: " << rotation_vector.angle() * 180 / M_PI << " axis is: " << rotation_vector.axis().transpose() << std::endl;

  //旋转向量转化为旋转矩阵
  Eigen::Matrix3d rotation_matrix3d = rotation_vector.matrix();
  std::cout<<"rotation_matrix3d \n"<< rotation_matrix3d << std::endl;
  //      cos45 -sin45         0
  //      sin45  cos45         0
  //      0         0          1

  //      0.707107 -0.707107         0
  //      0.707107  0.707107         0
  //      0         0                1

  // 旋转矩阵转换为欧拉角,"2" represents the z axis , "0" x axis, "1" y axis
  Eigen::Vector3d euler_angle = rotation_matrix3d.eulerAngles(0, 1, 2);
  std::cout << "绕z轴旋转的角度是 " << euler_angle.z() << std::endl;
  std::cout << "绕z轴旋转的角度是 " << euler_angle.z() * 180 / M_PI << std::endl;

  tf::Quaternion q;
  q.setRPY(0, 0, euler_angle.z());
  // q.setRPY(0, 0, 0.785398);
  tf::Transform transform;
  transform.setRotation( q );
  transform.setOrigin(tf::Vector3(1.0, 0.0, 0.0));
                          
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;
  ros::Rate rate(10.0);
  tf::TransformBroadcaster transform_broadcaster;
  while (node.ok()){
    // 发布的话题是　/tf
    transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    rate.sleep();
  }
  return 0;
};