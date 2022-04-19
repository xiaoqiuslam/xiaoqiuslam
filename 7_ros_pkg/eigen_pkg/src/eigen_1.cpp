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

using namespace std;
using namespace Eigen;


int main(int argc, char** argv){
  ros::init(argc, argv, "eigen_1_node");
  ros::NodeHandle nh;
  // rostopic type /odom -> nav_msgs/Odometry
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odometry_topic", 50);
  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time = ros::Time::now();
  ros::Time last_time = ros::Time::now();
  ros::Rate rate(1.0);
  while(nh.ok()){
    ros::spinOnce();              
    current_time = ros::Time::now();
    // 初始x的位置
    double x = 1.0;
    // 初始y的位置
    double y = 0.0;
    // 初始z点的位置
    double z = 0.0;

    // 弧度 = 角度*M_PI/180
    // 角度 = 弧度*180/M_PI
    double yaw_z = 90*M_PI/180;

    // 0*M_PI/180
    double pitch_y = 0*M_PI/180;

    // 0*M_PI/180
    double roll_x = 0*M_PI/180;

    // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll_x, pitch_y, yaw_z);
    std::cout << "odom_quat: " << odom_quat.w << " " << odom_quat.x << odom_quat.y << odom_quat.z << endl;

    // 创建TransformStamped消息，通过tf发送odom为父坐标系,base_link为子坐标系
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "base_link";
    odom_trans.child_frame_id = "odometry";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = z;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // publish the odometry message over ROS 
    nav_msgs::Odometry odometry;
    odometry.header.stamp = current_time;
    odometry.header.frame_id = "base_link";

    // set the position
    odometry.pose.pose.position.x = x;
    odometry.pose.pose.position.y = y;
    odometry.pose.pose.position.z = z;
    odometry.pose.pose.orientation = odom_quat;

    // set the velocity 
    // x方向速度0.1m/s
    double vx = 0.1;
    // y方向速度-0.1m/s
    double vy = -0.1;
    // th方向角0.1rad/s
     double vth = 0.1;
    odometry.child_frame_id = "odometry";
    odometry.twist.twist.linear.x = vx;
    odometry.twist.twist.linear.y = vy;
    odometry.twist.twist.angular.z = vth;

    // publish the message
    odom_pub.publish(odometry);

    last_time = current_time;
    rate.sleep();
  }
}



