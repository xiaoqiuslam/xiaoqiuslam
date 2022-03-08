#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <iostream>							//标准C++库中的输入输出
#include <pcl/io/pcd_io.h>					//PCD文件的读写
#include <pcl/point_types.h>				//点类型定义
#include <pcl/point_cloud.h>				//点云类定义
#include <pcl/kdtree/kdtree_flann.h>		//KD-Tree搜索


/*common模块*/
#include <pcl/common/common.h>				//标准的C以及C++类，是其他common 函数的父类;getMinMax3D()函数所需头文件，获得点云三维坐标的最值
#include <pcl/common/angles.h>				//定义了标准的C接口的角度计算函数
#include <pcl/common/centroid.h>			//定义了中心点的估算以及协方差矩阵的计算
#include <pcl/common/distances.h>			//定义标准的C接口用于计算距离
#include <pcl/common/file_io.h>				//定义了一些文件帮助写或者读方面的功能。
#include <pcl/common/random.h>				//定义一些随机点云生成的函数
#include <pcl/common/geometry.h>			//定义一些基本的几何功能的函数
#include <pcl/common/intersections.h>		//定义了线与线相交的函数
#include <pcl/common/norms.h>				//定义了标准的C方法计算矩阵的正则化
#include <pcl/common/time.h>				//定义了时间计算的函数


/*surface模块*/
#include <pcl/surface/mls.h>				//最小二乘平滑处理
#include <pcl/surface/concave_hull.h>		//创建凹多边形
#include <pcl/surface/gp3.h>				//贪婪投影三角化算法
#include <pcl/surface/organized_fast_mesh.h>

/*feature模块*/
#include <pcl/features/normal_3d.h>			//法线特征估计
#include <pcl/features/normal_3d_omp.h>		//法线特征估计加速
#include <pcl/features/pfh.h>				//PFH特征估计
#include <pcl/features/fpfh.h>				//FPFH特征估计
#include <pcl/features/fpfh_omp.h>			//FPFH特征估计加速
#include <pcl/features/vfh.h>				//VFH特征估计
#include <pcl/features/narf.h>				//NARF特征估计
#include <pcl/features/boundary.h>			//边界提取
#include <pcl/features/integral_image_normal.h>

/*registration模块*/
#include <pcl/registration/icp.h>			//ICP配准
#include <pcl/registration/icp_nl.h>		//非线性ICP配准
#include <pcl/registration/ndt.h>			//NDT配准
#include <pcl/registration/transforms.h>	//变换矩阵
#include <pcl/registration/ia_ransac.h>		//sac-ia类头文件
#include <pcl/registration/correspondence_estimation.h>					//直方图配准
#include <pcl/registration/correspondence_rejection_features.h>			//特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h>	//随机采样一致性去除 

/*filters模块*/
#include <pcl/filters/filter.h>				//滤波相关头文件
#include <pcl/filters/passthrough.h>		//滤波相关类头文件
#include <pcl/filters/project_inliers.h>	//滤波相关类头文件，点云投影
#include <pcl/filters/extract_indices.h>	//索引提取
#include <pcl/filters/voxel_grid.h>			//基于体素网格化的滤波
#include <pcl/filters/approximate_voxel_grid.h>			//体素网格过滤器滤波
#include <pcl/filters/statistical_outlier_removal.h>	//统计离群点

/*segmentation模块*/
#include <pcl/ModelCoefficients.h>						//采样一致性模型
#include <pcl/sample_consensus/method_types.h>			//随机参数估计方法
#include <pcl/sample_consensus/model_types.h>			//模型定义
#include <pcl/segmentation/sac_segmentation.h>			//基于采样一致性分割
#include <pcl/segmentation/region_growing.h>			//区域生长头文件
#include <pcl/segmentation/region_growing_rgb.h>		//基于颜色的区域生长
#include <pcl/segmentation/supervoxel_clustering.h>		//超体聚类
#include <pcl/segmentation/lccp_segmentation.h>			//基于凹凸性分割

/*visualization模块*/
#include <pcl/visualization/cloud_viewer.h>				//CloudViewer类可视化
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

/*range_image模块*/
#include <pcl/range_image/range_image.h>		//深度图像相关
#include <pcl/range_image/range_image_planar.h>

/*Eigen模块*/
#include <Eigen/StdVector>
#include <Eigen/Geometry>

/*console模块*/
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/console/parse.h>


#include <pcl/io/io.h>	//IO相关头文件
#include <boost/make_shared.hpp>			//boost指针相关头文件
#include <pcl/point_representation.h>		//点表示相关头文件
#include <pcl/io/openni2_grabber.h>	//OpenNI数据流获取类相关头文件


int
main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> ("/home/q/xiaoqiuslamshizhanjiaocheng/pcl_stu/data/tutorials/table_scene_mug_stereo_textured.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_mug_stereo_textured_inliers.pcd", *cloud_filtered, false);

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("table_scene_mug_stereo_textured_outliers.pcd", *cloud_filtered, false);

  return (0);
}
