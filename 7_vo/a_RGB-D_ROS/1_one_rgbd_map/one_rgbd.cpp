#include <ros/ros.h> 
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <cmath>

// C++ 标准库
#include <iostream>
#include <string>
using namespace std;

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

// 定义点云类型
// pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

#define PI 3.1415926535

main (int argc, char **argv) 
{ 
    ros::init(argc, argv, "image_ponit_cloud_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("image_point_cloud_topic", 1);
    ros::Rate rate(3);
    sensor_msgs::PointCloud2 ros_point_cloud;
    while (ros::ok()){
        ros_point_cloud.header.stamp = ros::Time::now();
		cv::Mat rgb, depth;
		rgb = cv::imread( "/media/q/q/zero_start_slam_20220707/3_pcl&ros/src/pcl_pkg/image/rgb.png" );
		depth = cv::imread( "/media/q/q/zero_start_slam_20220707/3_pcl&ros/src/pcl_pkg/image/depth.png", -1 );
		PointCloud::Ptr cloud ( new PointCloud );
		for (int m = 0; m < depth.rows; m++)
			for (int n=0; n < depth.cols; n++){
				ushort d = depth.ptr<ushort>(m)[n];
				if (d == 0)
					continue;
				PointT p;
				p.z = double(d) / camera_factor;
				p.x = (n - camera_cx) * p.z / camera_fx;
				p.y = -((m - camera_cy) * p.z / camera_fy);
				p.b = rgb.ptr<uchar>(m)[n*3];
				p.g = rgb.ptr<uchar>(m)[n*3+1];
				p.r = rgb.ptr<uchar>(m)[n*3+2];
				cloud->points.push_back( p );
			}
		cloud->height = 1;
		cloud->width = cloud->points.size();
		cout<<"point cloud size = "<<cloud->points.size()<<endl;
		cloud->is_dense = false;

		 // 保存点云pcd
		pcl::io::savePCDFile( "../pointcloud.pcd", *cloud );
		cout<<"Point cloud saved."<<endl;

		// 可视化点云pcl
		pcl::visualization::CloudViewer viewer( "viewer" );
		viewer.showCloud( cloud );
		while( !viewer.wasStopped() ){
			// 发布点云话题
			pcl::toROSMsg(*cloud, ros_point_cloud);
			ros_point_cloud.header.frame_id = "image_point_cloud_link";
			pub.publish(ros_point_cloud);
			rate.sleep();
		}
	}
    ros::spin();
	return 0; 
}