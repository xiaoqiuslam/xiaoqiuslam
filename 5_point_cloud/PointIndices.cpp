#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char *argv[]){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("../DensePointCloud.pcd", *cloud);
    
    std::cout << "cloud" << *cloud << std::endl;
    std::cout << "0 " << cloud->at(0) << std::endl;
    std::cout << "1 " << cloud->at(10) << std::endl;
    std::cout << "2 " << cloud->at(100) << std::endl;

    pcl::PointIndices indices;
    indices.indices.push_back(0);
    indices.indices.push_back(10);
    indices.indices.push_back(100);
    pcl::IndicesPtr  indices_ptr(new std::vector<int>(indices.indices));

    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(indices_ptr);
    extract_indices.setNegative(false);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_indices(new pcl::PointCloud<pcl::PointXYZRGB>);
    extract_indices.filter(*cloud_indices);

    std::cout << "\ncloud_indices" << *cloud_indices << std::endl;
    std::cout << "0 " << cloud_indices->at(0) << std::endl;
    std::cout << "1 " << cloud_indices->at(1) << std::endl;
    std::cout << "2 " << cloud_indices->at(2) << std::endl;

    return 0;
}

