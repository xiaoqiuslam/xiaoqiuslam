#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  // 节点
  ros::init(argc, argv, "nav_msgs_odometry_tf_node");
  // 句柄
  ros::NodeHandle nh;
  // 话题 rostopic list -> /odom 
  // rostopic type /odom -> nav_msgs/Odometry
  ros::Publisher nav_msgs_odometry_publisher = nh.advertise<nav_msgs::Odometry>("nav_msgs_odometry", 50);
  // tf
  tf::TransformBroadcaster transform_broadcaster;
  // 初始x点的位置
  double x = 0.0;
  // 初始y点的位置
  double y = 0.0;
  // 初始弧度制角度值
  double th = 0.0;
  // x方向速度0.1m/s
  double vx = 0.1;
  // y方向速度-0.1m/s
  double vy = -0.1;
  // th方向角0.1rad/s
  double vth = 0.1;
  // base_link参考odom以x轴方向0.1m/s，Y轴速度-0.1m/s，角速度0.1rad/s的圆周运动
  // 时间
  ros::Time current_time = ros::Time::now();
  ros::Time last_time = ros::Time::now();
  // 以1Hz的速率发布里程计信息
  ros::Rate rate(1.0);
  while(nh.ok()){
    // check for incoming messages
    ros::spinOnce();              
    current_time = ros::Time::now();
    // compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    // x方向的距离是速度vx在x方向的分量减去vy在x方向的分量乘上时间
    // 这里考虑的是全向运动模型，如果是双轮机器人，只有x方向的速度
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    // y方向的距离是速度vx在y方向的分量加上vy在y方向的分量乘上时间
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    // 弧度制角度等于角速度乘时间
    double delta_th = vth * dt;

    // x方向的距离
    x += delta_x;
    // y方向的距离
    y += delta_y;
    // 弧度制角度
    th += delta_th;

    // odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(th);

    /**
    first, we'll publish the transform over tf
    创建TransformStamped消息，通过tf发送odom为父坐标系,base_link为子坐标系
    */
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = current_time;
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "base_link";
    transform_stamped.transform.translation.x = x;
    transform_stamped.transform.translation.y = y;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation = quaternion;
    // send the transform
    transform_broadcaster.sendTransform(transform_stamped);

    // publish the odometry message over ROS 使得导航从中获取速度信息
    nav_msgs::Odometry odometry;
    odometry.header.stamp = current_time;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "base_link";
    // set the position
    odometry.pose.pose.position.x = x;
    odometry.pose.pose.position.y = y;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = quaternion;
    // set the velocity 
    odometry.twist.twist.linear.x = vx;
    odometry.twist.twist.linear.y = vy;
    odometry.twist.twist.angular.z = vth;
    // publish the message
    nav_msgs_odometry_publisher.publish(odometry);

    last_time = current_time;
    rate.sleep();
  }
}



