#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    uint8_t r(255), g(15), b(15);
    for (float z(-1.0); z <= 1.0; z += 0.05)
    {
        for (float angle(0.0); angle <= 360.0; angle += 5.0)
        {
            pcl::PointXYZ basic_point;
            basic_point.x = 0.5 * cosf(pcl::deg2rad(angle));
            basic_point.y = sinf(pcl::deg2rad(angle));
            basic_point.z = z;
            basic_cloud_ptr->points.push_back(basic_point);

            pcl::PointXYZRGB point;
            point.x = basic_point.x;
            point.y = basic_point.y;
            point.z = basic_point.z;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                            static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            point.rgb = *reinterpret_cast<float *>(&rgb);
            point_cloud_ptr->points.push_back(point);
        }
        if (z < 0.0)
        {
            r -= 12;
            g += 12;
        }
        else
        {
            g -= 12;
            b += 12;
        }
    }

    pcl::visualization::PCLVisualizer viewer("two viewer");
    int v1(0); //设置左右窗口
    int v2(1);

    // (Xmin,Ymin,Xmax,Ymax)设置窗口坐标
    viewer.createViewPort(0.0, 0.0, 0.5, 1, v1);
    viewer.setBackgroundColor(0, 0, 0, v1);

    viewer.createViewPort(0.5, 0.0, 1, 1, v2);
    viewer.setBackgroundColor(0.5, 0.5, 0.5, v2);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(basic_cloud_ptr, 0, 0, 255);
    viewer.addPointCloud(basic_cloud_ptr, blue, "cloud1", v1);

//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green(point_cloud_ptr, 0, 255, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green(point_cloud_ptr);
    viewer.addPointCloud(point_cloud_ptr, "cloud2", v2);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}
