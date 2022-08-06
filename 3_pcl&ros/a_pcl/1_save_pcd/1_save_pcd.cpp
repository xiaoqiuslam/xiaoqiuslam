#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    // 点云遍历赋值方式一
    // for (auto &point : cloud)
    // {
    //     point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    //     point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    //     point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    // }

    // 点云遍历赋值方式二
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII("1_save_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.size() << " data points to 1_save_pcd.pcd." << std::endl;

    for (const auto &point : cloud)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    return (0);

}