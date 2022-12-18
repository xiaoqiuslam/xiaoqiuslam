#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


int main(int argc, char **argv)
{
  // rosrun b_tf static_tf_broadcaster_eigen2tf child_frame_name 1 0 0 0 0 0
  ros::init(argc,argv, "my_static_tf2_broadcaster");
  if(argc != 8){
    ROS_ERROR("usage: static_turtle_tf2_broadcaster child_frame_name x y z roll pitch yaw");
    return -1;
  }

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "world";
  static_transformStamped.child_frame_id = argv[1];
  static_transformStamped.transform.translation.x = atof(argv[2]);
  static_transformStamped.transform.translation.y = atof(argv[3]);
  static_transformStamped.transform.translation.z = atof(argv[4]);

  tf2::Quaternion quat;
  quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();

  static_broadcaster.sendTransform(static_transformStamped);

  ros::spin();

  return 0;
};

