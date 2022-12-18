#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;

  // 调用服务产生第二只乌龟turtle2
  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  // 声明控制turtle2运动的速度的发布器Publisher
  ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  // 创建一个tf::TransformListener类型的监听器
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    // try-catch结构，可以获取抛出的异常
    try{
      // 通过监听tf实时得到/turtle2坐标系到turtle1坐标系的变换
      listener.waitForTransform("/turtle2", "/turtle1", ros::Time(0), ros::Duration(10.0) );
      // 1. 定义监听器 tf::TransformListener listener
      // 2. 定义存放变换关系的变量 tf::StampedTransform transform
      // 3. lookupTransform()监听两个坐标系之间的变换，实现/turtle2坐标系到turtle1坐标系的变换
      // 4. ros::Time(0) 表示最新一次的坐标变换。
      listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);

    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    // 根据turtle2和turtle1之间的坐标系位置关系，计算turtle2需要运动的线速度和角速度,并发布速度控制指令，使turtle2向turtle1移动
    geometry_msgs::Twist vel_msg;
    // atan2() 是已知一个角的正切值（也就是 y/x），求该角的弧度值。 
    // https://blog.csdn.net/weixin_46098577/article/details/116026828
    // geometry_msgs/Twist->geometry_msgs/Vector3 angular->angular.z代表平面机器人的角速度，因为此时z轴为旋转轴
    // 角速度
    // pid * vel_msg.angular.z = atan2(transform.getOrigin().y(), transform.getOrigin().x());
    vel_msg.angular.z = atan2(transform.getOrigin().y(), transform.getOrigin().x());
    // pow(n,2)求n的平方
    // sqrt(n)求n的平方根
    // linear.x指向机器人前方，linear.y指向左方，linear.z垂直向上满足右手系，平面移动机器人常常linear.y和linear.z均为0
    // 线速度 
    // pid * vel_msg.linear.x = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
    vel_msg.linear.x = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);
    rate.sleep();
  }
  return 0;
};
