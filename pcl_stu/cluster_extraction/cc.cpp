#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/common/angles.h>
#include <pcl/visualization/cloud_viewer.h>


int main(int argc, char** argv){
    pcl::PCLPointCloud2::Ptr scene (new pcl::PCLPointCloud2);   
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);
    pcl::io::loadPCDFile("/home/q/xiaoqiuslamshizhanjiaocheng/pcl_stu/data/tutorials/table_scene_lms400.pcd", *cloud_filtered);
    pcl::PointIndices::Ptr tmpinliers(new pcl::PointIndices);
    double distance = 50, degree = 60, max = 10000;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Ncloud_ground_plane(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::VectorXf coefficients;
    pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>(cloud_filtered));
    model->setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));	//setAxis用于设置所搜索平面垂直的轴
    model->setEpsAngle(pcl::deg2rad(degree));		//setEpsAngle用于设置待检测的平面模型和上述轴的最大角度
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setMaxIterations(max);
    ransac.setDistanceThreshold(distance);
    ransac.computeModel();
    ransac.getInliers(tmpinliers->indices);
    ransac.getModelCoefficients(coefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(tmpinliers);
    extract.setNegative(true);
    extract.filter(*Ncloud_ground_plane);
    *cloud_filtered = *Ncloud_ground_plane;
    std::cout << "show the data after deleting ground plane!" << endl;
    /*pcl::PCDWriter writer;
    writer.write("downsampled.pcd", *cloud_filtered);*/
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh test"));    
    view->setBackgroundColor(0, 0, 0);      
    view->addPointCloud(cloud_filtered, "target_cloud_v1");
    while (!view->wasStopped())
    {
        view->spinOnce(100);    
        pcl_sleep(0.01);
    }

    return 0;
}
