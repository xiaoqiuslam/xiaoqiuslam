// C++ 标准库
#include <iostream>
using namespace std;

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

// 主函数
int main( int argc, char** argv )
{
    // 读取./data/rgb.png和./data/depth.png，并转化为点云
    // 图像矩阵
    cv::Mat rgb, depth;
    // 使用cv::imread()来读取图像
    rgb = cv::imread( "../rgb.png" );
    // rgb 图像是8UC3的彩色图像
    // depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
    depth = cv::imread( "../depth.png", -1 );

    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZRGB> );
    // 遍历深度图
    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            pcl::PointXYZRGB p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = -((m - camera_cy) * p.z / camera_fy);
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout<<"point cloud size = "<<cloud->points.size()<<endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile( "../one_rgbd.pcd", *cloud );



    pcl::visualization::PCLVisualizer viewer("cloud");
    // 设置点云显示的背景色为黑色
    viewer.setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> fildColor(cloud);
    /**
     * 想更新一个 已经显示的点云，必须先调用removePointCloud（），并提供需要更新的点云ID 号，
     */
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, fildColor, "sample");
    /**
 * //用于改变显示点云的尺寸，可以利用该方法控制点云在视窗中的显示方法，
viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
 */
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample"); // 设置点云大小

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    // 清除数据并退出
    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;
    return 0;
}