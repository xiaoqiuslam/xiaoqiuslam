#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
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
    basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
    basic_cloud_ptr->height = 1;
    basic_cloud_ptr->is_dense = false;
    pcl::io::savePCDFile( "../2_save_pcd_basic_cloud_ptr.pcd", *basic_cloud_ptr );
    for (const auto &point : basic_cloud_ptr->points)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    point_cloud_ptr->is_dense = false;
    pcl::io::savePCDFile( "../2_save_pcd_point_cloud_ptr.pcd", *point_cloud_ptr );
    for (const auto &point : point_cloud_ptr->points)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    return (0);

}