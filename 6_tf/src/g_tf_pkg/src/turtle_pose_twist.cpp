#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

void pub_pose(const turtlesim::Pose::ConstPtr &pose){
    ROS_INFO("小海龟的坐标：(%.2f,　%.2f),　朝向rad：%.2f,　线速度m/s：%.2f,　角速度rad/s：%.2f", pose->x-5.54, pose->y-5.54, pose->theta, pose->linear_velocity, pose->angular_velocity);
}

int main(int argc, char *argv[]){
    // 中文
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "twist_pose");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/turtle1/pose", 10, pub_pose);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Rate rate(10);
    geometry_msgs::Twist twist;
    // 直线运动
    twist.linear.x = 0.01;
    // 圆周运动
    // twist.linear.x = 0.05;
    // twist.angular.z = -0.05;
    while (ros::ok()){
        pub.publish(twist);
        ROS_INFO("小海龟的速度：(%.2f,　%.2f)",twist.linear.x, twist.angular.z);
        rate.sleep();
        ros::spinOnce();
    }
}