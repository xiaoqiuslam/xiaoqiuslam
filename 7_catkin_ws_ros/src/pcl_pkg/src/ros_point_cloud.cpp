#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "point_cloud_publisher");

  ros::NodeHandle n;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 50);

  unsigned int num_points = 100;

  int count = 0;
  ros::Rate r(1.0);
  while(n.ok()){
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "sensor_frame";

    cloud.points.resize(num_points);

    //we'll also add an intensity channel to the cloud
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(num_points);

    //generate some fake data for our point cloud
    for(unsigned int i = 0; i < num_points; ++i){
      cloud.points[i].x = 0.1 + 0.1*count;
      cloud.points[i].y = 0.2 + 0.1*count;
      cloud.points[i].z = 0;
      cloud.channels[0].values[i] = 100 + count;
    }

    cloud_pub.publish(cloud);
    ++count;
    r.sleep();
  }
}
