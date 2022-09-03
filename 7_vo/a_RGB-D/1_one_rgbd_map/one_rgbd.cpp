#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>

const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

int main( int argc, char** argv )
{
    // 读取rgb.png和depth.png并转化为点云
    cv::Mat rgb, depth;
    // rgb 图像是8UC3的彩色图像
    rgb = cv::imread( "../rgb.png" );
    // depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
    depth = cv::imread( "../depth.png", -1 );

    // 点云变量,使用智能指针,创建一个空点云,指针用完会自动释放
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZRGB> );
    // 遍历深度图
    for (int u = 0; u < depth.rows; u++)
        for (int v=0; v < depth.cols; v++)
        {
            // 获取深度图中(u,n)处的值
            ushort d = depth.ptr<ushort>(u)[v];

            // 过滤深度值等于数零的值
            if (d == 0)
                continue;

            pcl::PointXYZRGB p;
            // 计算点的三维空间坐标
            p.z = double(d) / camera_factor;
            p.x = (v - camera_cx) * p.z / camera_fx;
            p.y = -((u - camera_cy) * p.z / camera_fy);
            
            // 从rgb图像中获取它的颜色,rgb是三通道的BGR格式图,按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(u)[v*3];
            p.g = rgb.ptr<uchar>(u)[v*3+1];
            p.r = rgb.ptr<uchar>(u)[v*3+2];

            // p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout<<"point cloud size : "<<cloud->points.size()<<endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile( "../one_rgbd.pcd", *cloud );
    cout<<"Point cloud saved "<<endl;

    // 可视化点云
    pcl::visualization::PCLVisualizer viewer("cloud");
    viewer.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> fildColor(cloud);
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, fildColor, "sample");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    cloud->points.clear();
    return 0;
}