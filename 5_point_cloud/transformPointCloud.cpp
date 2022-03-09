#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>                  
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv){
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::io::loadPCDFile<pcl::PointXYZ> ("../table_scene_lms400.pcd", *source_cloud);
    /* transformation matrices work :
            |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)
    */
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    // rotation matrix: defined a 90° (PI/2) rotation around the Z axis and a translation on the X axis.
    float theta = M_PI/2; // angle of rotation in radians
    transformation (0,0) = cos (theta);
    transformation (0,1) = -sin(theta);
    transformation (1,0) = sin (theta);
    transformation (1,1) = cos (theta); 
    // translation 0 meters on the x, y, z axis.
    transformation (0,3) = 0; //(row, column)
    transformation (1,3) = 0; //(row, column)
    transformation (2,3) = 0; //(row, column)
    // Print the transformation
    printf ("using a Matrix4f\n");
    std::cout << transformation << std::endl;

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // Apply an affine transform defined by an Eigen Transform.
    pcl::transformPointCloud (*source_cloud, *transformed_cloud, transformation);

    // Visualization
    printf(  "\nPoint cloud colors :  white  = original point cloud\n"
      "                        red  = transformed point cloud\n");
    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
    viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");
    viewer.addCoordinateSystem (1.0, 0);  //Adds 3D axes describing a coordinate system to screen at 0,0,0. 
    viewer.initCameraParameters();
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    viewer.setPosition(800, 400); // Setting visualiser window position
    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
      viewer.spinOnce ();
    }

  return 0;
}