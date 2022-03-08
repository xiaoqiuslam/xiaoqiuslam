#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char *argv[]){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../table_scene_lms400.pcd", *cloud);
    std::cout << "cloud size " <<  cloud->size () << std::endl;
    return 0;
}