#include <cstdio>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{

  // rosrun b_tf static_tf_broadcaster_setRPY child_frame_name 1 0 0 0 0 0
  ros::init(argc,argv, "my_tf2_broadcaster");
  if(argc != 7)
  {
    ROS_ERROR("usage: tf2_broadcaster x y z roll pitch yaw");
    return -1;
  }

  ros::NodeHandle node;


  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.frame_id = "world";
  static_transformStamped.child_frame_id = "turtle1";
  static_transformStamped.transform.translation.x = atof(argv[1]);
  static_transformStamped.transform.translation.y = atof(argv[2]);
  static_transformStamped.transform.translation.z = atof(argv[3]);

  tf2::Quaternion quat;
  quat.setRPY(atof(argv[4]), atof(argv[5]), atof(argv[6]));
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();


  geometry_msgs::TransformStamped transformStamped_carrot1;
  transformStamped_carrot1.header.frame_id = "turtle1";
  transformStamped_carrot1.child_frame_id = "carrot1";
  transformStamped_carrot1.transform.translation.x = 0.0;
  transformStamped_carrot1.transform.translation.y = 2.0;
  transformStamped_carrot1.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped_carrot1.transform.rotation.x = q.x();
  transformStamped_carrot1.transform.rotation.y = q.y();
  transformStamped_carrot1.transform.rotation.z = q.z();
  transformStamped_carrot1.transform.rotation.w = q.w();

  geometry_msgs::TransformStamped transformStamped_carrot2;
  transformStamped_carrot2.header.frame_id = "turtle1";
  transformStamped_carrot2.child_frame_id = "carrot2";
  transformStamped_carrot2.transform.translation.x = 2.0*sin(ros::Time::now().toSec());
  transformStamped_carrot2.transform.translation.y = 2.0*cos(ros::Time::now().toSec());
  transformStamped_carrot2.transform.translation.z = 0.0;

  tf2::Quaternion q_carrot2;
  q_carrot2.setRPY(0, 0, 0);
  transformStamped_carrot2.transform.rotation.x = q_carrot2.x();
  transformStamped_carrot2.transform.rotation.y = q_carrot2.y();
  transformStamped_carrot2.transform.rotation.z = q_carrot2.z();
  transformStamped_carrot2.transform.rotation.w = q_carrot2.w();

  ros::Time::init();
  ros::Rate rate(1.0);
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  tf2_ros::TransformBroadcaster tfb;
  while (node.ok()){
    static_transformStamped.header.stamp = ros::Time::now();
    transformStamped_carrot1.header.stamp = ros::Time::now();
    transformStamped_carrot2.header.stamp = ros::Time::now();
    // 发布的话题是　/tf_static
    static_broadcaster.sendTransform(static_transformStamped);

    // 发布的话题是　/tf
    tfb.sendTransform(transformStamped_carrot1);

    // 发布的话题是　/tf
    tfb.sendTransform(transformStamped_carrot2);

    rate.sleep();
  }
  ros::spin();
  return 0;
};
