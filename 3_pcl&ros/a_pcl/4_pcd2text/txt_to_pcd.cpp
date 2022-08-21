#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr BaseMeasure::readCloudTxt(char* Filename)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	ifstream Points_in(Filename);
	pcl::PointXYZ tmpoint;
	if (Points_in.is_open())
	{
		while (!Points_in.eof())   //尚未到达文件结尾
		{
			Points_in >> tmpoint.x >> tmpoint.y >> tmpoint.z;
			basic_cloud_ptr->points.push_back(tmpoint);
		}
	}
	basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
	basic_cloud_ptr->height = 1;
 
	return basic_cloud_ptr;
}