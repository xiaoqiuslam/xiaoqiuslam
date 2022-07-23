#include <iostream>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "point_cloud_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_pub_topic", 100);

    ros::Rate rate(3);
    unsigned int num_points = 3;

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.points.resize(num_points);
    sensor_msgs::PointCloud2 output_msg;

    while (ros::ok())
    {
        output_msg.header.stamp = ros::Time::now();
        for(int i = 0; i < num_points; i++){
            cloud.points[i].x = i;
            cloud.points[i].y = i;
            cloud.points[i].z = i;

            cloud.points[i].r = 0;
            cloud.points[i].g = 255;
            cloud.points[i].b = 0;

        }
        pcl::toROSMsg(cloud, output_msg);
        output_msg.header.frame_id = "point_cloud_frame_id";
        pub.publish(output_msg);
        rate.sleep();
    }
    ros::spin();
    return 0;
}