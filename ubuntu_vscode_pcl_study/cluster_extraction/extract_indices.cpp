#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#define BOOST_TYPEOF_EMULATION
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <boost/thread/thread.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/angles.h>
using namespace pcl::console;
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
double tstart, tstop, ttime;
std::vector<std::string> pcd_files_;
std::vector<boost::filesystem::path> pcd_paths_;
std::string dir_;
boost::shared_ptr<pcl::visualization::PCLVisualizer> p;
int vp_1, vp_2, vp_3;
int cidx = -100;

int main (){
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
  pcl::PCDReader reader;
  reader.read ("/home/q/xiaoqiuslamshizhanjiaocheng/pcl_stu/data/tutorials/table_scene_lms400.pcd", *cloud_blob);
  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  sor.filter (*cloud_filtered_blob);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Ncloud_ground_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
  
  pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGB>::Ptr model(new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGB>(cloud_filtered_blob));
  model->setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));   //设置所搜索平面垂直的轴 
  model->setEpsAngle(pcl::deg2rad(25));         //设置待检测的平面模型和上述轴的最大角度
  
  pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
  ransac.setMaxIterations(10000);    //最大迭代次数
  ransac.setDistanceThreshold(0.06);  //距离阈值
  ransac.computeModel();

  pcl::PointIndices::Ptr tmpinliers(new pcl::PointIndices);
  ransac.getInliers(tmpinliers->indices);
  Eigen::VectorXf coefficients;
  ransac.getModelCoefficients(coefficients);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud_filtered_blob);
  extract.setIndices(tmpinliers);
  extract.setNegative(true);
  extract.filter(*Ncloud_ground_plane);
  pcl::PCDWriter writer;
  writer.write ("table_scene_mug_stereo_textured_downsampled.pcd", *Ncloud_ground_plane, false);

  return (0);
}