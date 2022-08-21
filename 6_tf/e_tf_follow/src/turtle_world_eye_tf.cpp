#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include "turtlesim/Pose.h"
#include "tf/transform_broadcaster.h"
// #include "geometry_msgs/TransformStamped.h"
// #include "tf/LinearMath/Quaternion.h"


// 获取turtle1的位姿打印在屏幕上然后发布turtle1坐标系相对于world坐标系的tf
void turtle1_pose(const turtlesim::Pose::ConstPtr & pose){
    ROS_INFO("turtle1的坐标：(%.2f,　%.2f),　朝向rad：%.2f,　线速度m/s：%.2f,　角速度rad/s：%.2f", pose->x, pose->y, pose->theta, pose->linear_velocity, pose->angular_velocity);  
    static tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = "world";
    tf.header.stamp = ros::Time::now();

    tf.child_frame_id = "turtle1";
    tf.transform.translation.x = pose->x;
    tf.transform.translation.y = pose->y;

    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, pose->theta);
    tf.transform.rotation.x = quaternion.getX(); 
    tf.transform.rotation.y = quaternion.getY(); 
    tf.transform.rotation.z = quaternion.getZ(); 
    tf.transform.rotation.w = quaternion.getW(); 

    broadcaster.sendTransform(tf);
    ROS_INFO("发布turtle1相对于world的坐标系关系");

    // turtle1的眼睛，也就是相机之间是静态坐标系关系，二者相对位置不会改变
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.05, 0.0, 0.1)), ros::Time::now(),"turtle1", "turtle1_eye"));
    ROS_INFO("发布相机相对于turtle1的坐标系关系");
}

// 获取turtle2的位姿打印在屏幕上然后发布turtle2坐标系相对于world坐标系的tf
void turtle2_pose(const turtlesim::Pose::ConstPtr & pose){
    ROS_INFO("turtle2的坐标：(%.2f,　%.2f),　朝向rad：%.2f,　线速度m/s：%.2f,　角速度rad/s：%.2f", pose->x, pose->y, pose->theta, pose->linear_velocity, pose->angular_velocity);
    static tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tf;

    tf.header.frame_id = "world";
    tf.header.stamp = ros::Time::now();

    tf.child_frame_id = "turtle2";
    tf.transform.translation.x = pose->x;
    tf.transform.translation.y = pose->y;

    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, pose->theta);
    tf.transform.rotation.x = quaternion.getX(); 
    tf.transform.rotation.y = quaternion.getY(); 
    tf.transform.rotation.z = quaternion.getZ(); 
    tf.transform.rotation.w = quaternion.getW(); 


    broadcaster.sendTransform(tf);
    ROS_INFO("发布turtle2相对于world的坐标系关系");

    // turtle2的眼睛，也就是相机之间是静态坐标系关系，二者相对位置不会改变
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.05, 0.0, 0.1)), ros::Time::now(),"turtle2", "turtle2_eye"));
    ROS_INFO("发布相机相对于turtle2的坐标系关系");
}

int main(int argc, char** argv){
  setlocale(LC_ALL,"");
  ros::init(argc, argv, "turtle_tf");
  ros::NodeHandle nh;

  // 订阅turtle1在world下的位置姿态，然后发布turtle1相对world的坐标关系
  ros::Subscriber turtle1_sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 100, turtle1_pose);

  // 订阅turtle1在world下的位置姿态，然后发布turtle２相对world的坐标关系
  ros::Subscriber turtle2_sub = nh.subscribe<turtlesim::Pose>("/turtle2/pose", 100, turtle2_pose);

  ros::spin();
}