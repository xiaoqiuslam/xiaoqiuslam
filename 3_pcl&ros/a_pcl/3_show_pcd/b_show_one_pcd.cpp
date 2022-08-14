#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>

int main()
{
    // 定义PCLVisualizer点云显示对象
    pcl::visualization::PCLVisualizer viewer;
     // 设置背景颜色
    viewer.setBackgroundColor(0, 0, 0);
    // 在原点添加坐标系轴 3.0指轴的长度
    viewer.addCoordinateSystem(3.0); 
    //viewer.addCoordinateSystem (3.0,1,2,3);一个重载函数，3.0指轴的长度，放置在（1，2，3）位置
    // 初始化相机参数
    viewer.initCameraParameters();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::io::loadPCDFile("../cat.pcd", *cloud);                                          
    // 设置显示点云的颜色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, red, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 30, "cloud");
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}