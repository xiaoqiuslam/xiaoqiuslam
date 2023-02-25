#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the camera_link frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped camera_point;
  camera_point.header.frame_id = "camera_link";

  //we'll just use the most recent transform available for our simple example
  camera_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  camera_point.point.x = 1.0;
  camera_point.point.y = 0.2;
  camera_point.point.z = 0.0;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("world", camera_point, base_point);

    ROS_INFO("camera_link: (%.2f, %.2f. %.2f) -----> world: (%.2f, %.2f, %.2f) at time %.2f",
        camera_point.point.x, camera_point.point.y, camera_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"camera_link\" to \"world\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}
