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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_change(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("../DensePointCloud.pcd", *cloud);
    // p.x = z;
    // p.z = -(y);
    // p.y = -(x);
    pcl::PointXYZRGB point;
    for (int i =  0; i <cloud->size(); i ++ ){
        point.x = cloud->at(i).z;
        point.z = -1.0*cloud->at(i).y;
        point.y = -1.0*cloud->at(i).x;
        point.r = cloud->at(i).r;
        point.g = cloud->at(i).g;
        point.b = cloud->at(i).b;
        cloud_change->push_back(point);
    }


    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
    viewer.addPointCloud (cloud, "original_cloud");
    viewer.addPointCloud (cloud_change, "transformed_cloud");
    viewer.addCoordinateSystem (1.0, 0);  //Adds 3D axes describing a coordinate system to screen at 0,0,0. 
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }

    return 0;
}

