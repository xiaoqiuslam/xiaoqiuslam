#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("../table_scene_mug_stereo_textured.pcd", *cloud);
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    pcl::visualization::PCLVisualizer viewer("two viewer");

    int v1(0); //设置左右窗口
    int v2(1);

    // (Xmin,Ymin,Xmax,Ymax)设置窗口坐标
    viewer.createViewPort(0.0, 0.0, 0.5, 1, v1); 
    viewer.setBackgroundColor(0, 0, 0, v1);

    viewer.createViewPort(0.5, 0.0, 1, 1, v2);
    viewer.setBackgroundColor(0.5, 0.5, 0.5, v2);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(cloud, 0, 0, 255);
    viewer.addPointCloud(cloud, blue, "cloud1", v1);

    pcl::PassThrough<pcl::PointXYZ> pass; //创建滤波器 pass
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.5);
    pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered);
    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud_filtered, 250, 0, 0);
    viewer.addPointCloud(cloud_filtered, red, "cloud2", v2);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}
