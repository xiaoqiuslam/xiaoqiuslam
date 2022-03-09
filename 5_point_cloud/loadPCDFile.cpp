#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char *argv[]){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../table_scene_lms400.pcd", *cloud);
    std::cout << "cloud size " <<  cloud->size () << std::endl;

    //根据想要的点添加到自定义的indices_0的数组中，
    //std::vector<int> indices_0;
    pcl::PointIndices indices_0;

    indices_0.indices.push_back(0);
    indices_0.indices.push_back(10);
    indices_0.indices.push_back(100);
    //将自定义的indices_0数组进行智能指针的转化
    //pcl::IndicesPtr  index_ptr_0 = boost::make_shared<std::vector<int>>(indices_0.indices);
    pcl::IndicesPtr  index_ptr_0(new std::vector<int>(indices_0.indices));

    //利用ExtractIndices根据索引进行点云的提取
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_0);
    extract.setIndices(index_ptr_0);
    extract.setNegative(false);//如果设为true,可以提取指定index之外的点云
    extract.filter(*cloud_1);

    //cloud_1索引0，1，2分别对应与cloud_0索引的0，10，100
    std::cout << *cloud_1 << std::endl;
    std::cout << cloud_1->at(0) << std::endl;
    std::cout << cloud_1->at(1) << std::endl;
    std::cout << cloud_1->at(2) << std::endl;

    std::cout << *cloud_0 << std::endl;
    std::cout << cloud_0->at(0) << std::endl;
    std::cout << cloud_0->at(10) << std::endl;
    std::cout << cloud_0->at(100) << std::endl;

    return 0;
}

