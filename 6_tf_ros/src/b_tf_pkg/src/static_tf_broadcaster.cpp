#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


int main(int argc, char **argv)
{
  ros::init(argc,argv, "my_static_tf2_broadcaster");

  if(argc != 9){
    ROS_ERROR("usage: static_tf_broadcaster x y z roll pitch yaw map base_link");
    // static_tf_broadcaster 0.011 0.048 0.015 0 0 0 map base_link
    // rosrun tf static_transform_publisher 0.011 0.048 0.015 0 0 0 map base_link 100
    return -1;
  }

  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped geometry_msgs_transform_stamped;
  geometry_msgs_transform_stamped.header.stamp = ros::Time::now();
  geometry_msgs_transform_stamped.header.frame_id = argv[7];
  geometry_msgs_transform_stamped.child_frame_id = argv[8];
  geometry_msgs_transform_stamped.transform.translation.x = atof(argv[1]);
  geometry_msgs_transform_stamped.transform.translation.y = atof(argv[2]);
  geometry_msgs_transform_stamped.transform.translation.z = atof(argv[3]);
  tf2::Quaternion quat;
  quat.setRPY(atof(argv[4]), atof(argv[5]), atof(argv[6]));
  geometry_msgs_transform_stamped.transform.rotation.x = quat.x();
  geometry_msgs_transform_stamped.transform.rotation.y = quat.y();
  geometry_msgs_transform_stamped.transform.rotation.z = quat.z();
  geometry_msgs_transform_stamped.transform.rotation.w = quat.w();
  static_broadcaster.sendTransform(geometry_msgs_transform_stamped);

  ros::spin();

  return 0;
};



