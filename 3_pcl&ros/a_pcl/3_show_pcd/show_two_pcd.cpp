#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

int main()
{
    // 读取点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../cat.pcd", *cloud1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../horse.pcd", *cloud2);
    // 定义PCLVisualizer点云显示对象
    pcl::visualization::PCLVisualizer viewer;
    // 设置背景颜色
    viewer.setBackgroundColor(0, 0, 0);
    // 在原点添加坐标系轴 3.0指轴的长度
    viewer.addCoordinateSystem(3.0); 
    // 设置显示点云的颜色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(cloud1, 0, 255, 0);
    viewer.addPointCloud(cloud1, blue, "cloud1");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud2, 255, 0, 0);
    viewer.addPointCloud(cloud2, red, "cloud2");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
//        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    system("pause");
    return 0;
}