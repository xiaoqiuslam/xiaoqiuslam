#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle nh;
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);

  double laser_frequency = 40;
  double ranges[100];
  double intensities[100];
  int count = 0;

  ros::Rate rate(1.0);
  while(nh.ok()){
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < 100; ++i){
      ranges[i] = count;
      intensities[i] = 100 + count;
    }

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "laser_frame";
    // 负45度
    scan.angle_min = -0.785;
    // 正45度
    scan.angle_max = 0.785;
    // 角度递增 
    scan.angle_increment = 3.14 / 10;
    scan.time_increment = (1 / laser_frequency) / (10);
    // 激光束的最近距离
    scan.range_min = 0.0;
    // 激光束的最远距离
    scan.range_max = 100.0;
    scan.ranges.resize(100);
    scan.intensities.resize(100);
    for(unsigned int i = 0; i < 100; ++i){
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }
    scan_pub.publish(scan);
    ++count;
    rate.sleep();
  }
}
