#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {
    // Here we create three visualization_msgs/Marker messages and initialize all of their shared data. 
    // We take advantage of the fact that message members default to 0 and only set the w member of the pose. 
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;


    // We assign three different ids to the three markers. 
    // The use of the points_and_lines namespace ensures they won't collide with other broadcasters. 
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;


    // Here we set the marker types to POINTS, LINE_STRIP and LINE_LIST. 
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;


    // The scale member means different things for these marker types. 
    // The POINTS marker uses the x and y members for width and height respectively, 
    // while the LINE_STRIP and LINE_LIST markers only use the x component, 
    // which defines the line width. Scale values are in meters. 
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;



    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;


    // We use sine and cosine to generate a helix. 
    // The POINTS and LINE_STRIP markers both require only a point for each vertex, 
    // while the LINE_LIST marker requires 2. 
    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 10; ++i)
    {
      float y = 5 * sin(f + i / 10.0f * 2 * M_PI);
      float z = 5 * cos(f + i / 10.0f * 2 * M_PI);

      geometry_msgs::Point p;
      p.x = (int32_t)i - 50;
      p.y = y;
      p.z = z;

      points.points.push_back(p);
      line_strip.points.push_back(p);

      // The line list needs two points for each line
      line_list.points.push_back(p);
      p.z += 1.0;
      line_list.points.push_back(p);
    }


    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);

    r.sleep();

    f += 0.04;
  }
}