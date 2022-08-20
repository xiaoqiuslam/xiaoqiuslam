#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  // initialize ROS, and create a ros::Publisher on the visualization_marker topic. 
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  

  // Set our initial shape type to be a cube 
  // create an integer to keep track of what shape we're going to publish. 
  // The four types we'll be using here all use the visualization_msgs/Marker message in the same way, 
  // so we can simply switch out the shape type to demonstrate the four different shapes. 
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    // This begins the meat of the program. 
    // First we create a visualization_msgs/Marker, and begin filling it out. 
    // The header here is a roslib/Header, which should be familiar if you've done the tf tutorials. 
    // We set the frame_id member to /my_frame as an example. 
    // In a running system this should be the frame relative to which you want the marker's pose to be interpreted. 
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    // The namespace (ns) and id are used to create a unique name for this marker. 
    // If a marker message is received with the same ns and id, the new marker will replace the old one. 
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    // This type field is what specifies the kind of marker we're sending. 
    // The available types are enumerated in the visualization_msgs/Marker message. 
    // Here we set the type to our shape variable, which will change every time through the loop. 
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    // The action field is what specifies what to do with the marker. 
    // The options are visualization_msgs::Marker::ADD and visualization_msgs::Marker::DELETE. 
    // ADD is something of a misnomer, it really means "create or modify".
    // New in Indigo A new action has been added to delete all markers in the particular Rviz display, regardless of ID or namespace. 
    // The value is 3 and in future ROS version the message will change to have value visualization_msgs::Marker::DELETEALL. 
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    // Here we set the pose of the marker. 
    // The geometry_msgs/Pose message consists of a geometry_msgs/Vector3 to specify the position and a geometry_msgs/Quaternion to specify the orientation. 
    // Here we set the position to the origin, and the orientation to the identity orientation (note the 1.0 for w). 
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    // Now we specify the scale of the marker. 
    // For the basic shapes, a scale of 1 in all directions means 1 meter on a side. 
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    // The color of the marker is specified as a std_msgs/ColorRGBA. 
    // Each member should be between 0 and 1. An alpha (a) value of 0 means completely transparent (invisible), and 1 is completely opaque. 
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    // The lifetime field specifies how long this marker should stick around before being automatically deleted. 
    // A value of ros::Duration() means never to auto-delete.
    // If a new marker message is received before the lifetime has been reached, the lifetime will be reset to the value in the new marker message. 
    marker.lifetime = ros::Duration();

    // Publish the marker
    // We wait for the marker to have a subscriber and we then publish the marker. 
    // Note that you can also use a latched publisher as an alternative to this code. 
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);


    // Cycle between different shapes
    // This code lets us show all four shapes while just publishing the one marker message. 
    // Based on the current shape, we set what the next shape to publish will be. 
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }

    // Sleep and loop back to the top. 
    r.sleep();
  }
}
