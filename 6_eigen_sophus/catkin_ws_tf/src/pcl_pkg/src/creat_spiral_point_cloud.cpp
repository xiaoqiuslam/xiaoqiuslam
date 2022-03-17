#include <ros/ros.h> 
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <cmath>

#define PI 3.1415926535

main (int argc, char **argv) 
{ 
    long point_num;
	ros::init (argc, argv, "spiral_point_cloud_node"); 
	ros::NodeHandle nh; 
	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("spiral_point_cloud_topic", 1);     
	pcl::PointCloud<pcl::PointXYZ> cloud; 
	sensor_msgs::PointCloud2 output; 
	
	// Fill in the cloud data 
	cloud.width = 360; 
	cloud.height = 21; 
	cloud.points.resize(cloud.width * cloud.height); 

	for(int z=0;z<20;++z)
	{
		for(int loop=0;loop<360;++loop)
		{
			point_num = z*360 + loop;
			float hudu = 180 *loop/PI;
			cloud.points[point_num].x = cos(hudu); 
			cloud.points[point_num].y = sin(hudu); 
			// 或者为0.3*static_cast<float>(z);目地在于int转换为float
			cloud.points[point_num].z = 0.3*(z+1.0f);
		}	
	}
		
	//Convert the cloud to ROS message 
	pcl::toROSMsg(cloud, output); 
	output.header.frame_id = "odom"; 
	
	ros::Rate loop_rate(1); 
	while (ros::ok()) 
	{ 
		pcl_pub.publish(output);
		ros::spinOnce(); 
		loop_rate.sleep(); 
	 } 
	return 0; 
}