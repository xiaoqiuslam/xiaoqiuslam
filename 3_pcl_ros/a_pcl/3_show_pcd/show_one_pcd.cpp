#include <pcl/visualization/cloud_viewer.h> 
#include <iostream>                        
#include <pcl/io/io.h>                     
#include <pcl/io/pcd_io.h>                 

int main()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile("../room_scan.pcd", *cloud); 
    // 创建CloudViewer点云显示对象
    pcl::visualization::CloudViewer viewer("Cloud Viewer"); 
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
    }
    return 0;
}
