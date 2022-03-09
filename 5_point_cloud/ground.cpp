

#include "include/OccupancyGrid.hpp"
#include <pcl/io/pcd_io.h>// 文件、设备读写
#include <pcl/point_cloud.h>//基础pcl点云类型
#include <pcl/correspondence.h>//分组算法 对应表示两个实体之间的匹配（例如，点，描述符等）。
// 特征
#include <pcl/features/normal_3d_omp.h>//法向量特征
#include <pcl/features/shot_omp.h> //描述子 shot描述子 0～1
// https://blog.csdn.net/bengyanluo1542/article/details/76061928?locationNum=9&fps=1
// (Signature of Histograms of OrienTations)方向直方图特征
#include <pcl/features/board.h>
// 滤波
#include <pcl/filters/uniform_sampling.h>//均匀采样 滤波
// 识别
#include <pcl/recognition/cg/hough_3d.h>//hough算子
#include <pcl/recognition/cg/geometric_consistency.h> //几何一致性
// 可视化
#include <pcl/visualization/pcl_visualizer.h>//可视化
// kdtree
#include <pcl/kdtree/kdtree_flann.h>// kdtree 快速近邻搜索
#include <pcl/kdtree/impl/kdtree_flann.hpp>
// 转换
#include <pcl/common/transforms.h>//点云转换 转换矩阵
// 命令行参数
#include <pcl/console/parse.h>//命令行参数解析

typedef pcl::PointXYZRGB PointT; 

int main(int argc, char *argv[]){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZRGB>);
    

    // util3d::segmentObstaclesFromGround<pcl::PointXYZ>(cloud, ground, obstacles, 20, M_PI/4.0f, 0.02, 200, true);

    pcl::IndicesPtr ground;
    ground.reset(new std::vector<int>);

	pcl::IndicesPtr obstacles;
    obstacles.reset(new std::vector<int>);

	pcl::IndicesPtr * flatObstacles,
	flatObstacles->reset(new std::vector<int>);

    // Find the ground
	// pcl::IndicesPtr flatSurfaces = normalFiltering(cloud, indices, groundNormalAngle, Eigen::Vector4f(0,0,1,0), normalKSearch,viewPoint, groundNormalsUp);
    // flatSurfaces =  output;
    pcl::IndicesPtr output(new std::vector<int>());
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>(false));
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("../DensePointCloud.pcd", *cloud);
    ne.setInputCloud (cloud);
    pcl::PointIndices indices;
    tree->setInputCloud(cloud);
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(normalKSearch);
    ne.compute (*cloud_normals);
    output->resize(cloud_normals->size());
    int oi = 0; // output iterator
    for(unsigned int i=0; i<cloud_normals->size(); ++i)
    {
        Eigen::Vector4f v(cloud_normals->at(i).normal_x, cloud_normals->at(i).normal_y, cloud_normals->at(i).normal_z, 0.0f);
        if(groundNormalsUp>0.0f && v[2] < -groundNormalsUp && cloud->at(indices->size()!=0?indices->at(i):i).z < viewpoint[3]) // some far velodyne rays on road can have normals toward ground
        {
            //reverse normal
            v *= -1.0f;
        }

        float angle = pcl::getAngle3D(normal, v);
        if(angle < angleMax)
        {
            output->at(oi++) = indices->size()!=0?indices->at(i):i;
        }
    }
    output->resize(oi);


    

pcl::IndicesPtr normalFilteringImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float angleMax,
		const Eigen::Vector4f & normal,
		int normalKSearch,
		const Eigen::Vector4f & viewpoint,
		float groundNormalsUp)
{
	

	if(cloud->size())
	{

		
	}

	return output;
}

    

void segmentObstaclesFromGround(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const typename pcl::IndicesPtr & indices,
		int normalKSearch,
		float groundNormalAngle,
		float clusterRadius,
		int minClusterSize,
		bool segmentFlatObstacles,
		float maxGroundHeight,
		const Eigen::Vector4f & viewPoint,
		float groundNormalsUp)
{

		if(segmentFlatObstacles && flatSurfaces->size())
		{
			int biggestFlatSurfaceIndex;
			std::vector<pcl::IndicesPtr> clusteredFlatSurfaces = extractClusters(
					cloud,
					flatSurfaces,
					clusterRadius,
					minClusterSize,
					std::numeric_limits<int>::max(),
					&biggestFlatSurfaceIndex);

			// cluster all surfaces for which the centroid is in the Z-range of the bigger surface
			if(clusteredFlatSurfaces.size())
			{
				Eigen::Vector4f biggestSurfaceMin,biggestSurfaceMax;
				if(maxGroundHeight != 0.0f)
				{
					// Search for biggest surface under max ground height
					size_t points = 0;
					biggestFlatSurfaceIndex = -1;
					for(size_t i=0;i<clusteredFlatSurfaces.size();++i)
					{
						Eigen::Vector4f min,max;
						pcl::getMinMax3D(*cloud, *clusteredFlatSurfaces.at(i), min, max);
						if(min[2]<maxGroundHeight && clusteredFlatSurfaces.size() > points)
						{
							points = clusteredFlatSurfaces.at(i)->size();
							biggestFlatSurfaceIndex = i;
							biggestSurfaceMin = min;
							biggestSurfaceMax = max;
						}
					}
				}
				else
				{
					pcl::getMinMax3D(*cloud, *clusteredFlatSurfaces.at(biggestFlatSurfaceIndex), biggestSurfaceMin, biggestSurfaceMax);
				}
				if(biggestFlatSurfaceIndex>=0)
				{
					ground = clusteredFlatSurfaces.at(biggestFlatSurfaceIndex);
				}

				if(!ground->empty() && (maxGroundHeight == 0.0f || biggestSurfaceMin[2] < maxGroundHeight))
				{
					for(unsigned int i=0; i<clusteredFlatSurfaces.size(); ++i)
					{
						if((int)i!=biggestFlatSurfaceIndex)
						{
							Eigen::Vector4f centroid(0,0,0,1);
							pcl::compute3DCentroid(*cloud, *clusteredFlatSurfaces.at(i), centroid);
							if(maxGroundHeight==0.0f || centroid[2] <= maxGroundHeight || centroid[2] <= biggestSurfaceMax[2]) // epsilon
							{
								ground = util3d::concatenate(ground, clusteredFlatSurfaces.at(i));
							}
							else if(flatObstacles)
							{
								*flatObstacles = util3d::concatenate(*flatObstacles, clusteredFlatSurfaces.at(i));
							}
						}
					}
				}
				else
				{
					// reject ground!
					ground.reset(new std::vector<int>);
					if(flatObstacles)
					{
						*flatObstacles = flatSurfaces;
					}
				}
			}
		}
		else
		{
			ground = flatSurfaces;
		}

		if(ground->size() != cloud->size())
		{
			// Remove ground
			pcl::IndicesPtr notObstacles = ground;
			if(indices->size())
			{
				notObstacles = util3d::extractIndices(cloud, indices, true);
				notObstacles = util3d::concatenate(notObstacles, ground);
			}
			pcl::IndicesPtr otherStuffIndices = util3d::extractIndices(cloud, notObstacles, true);

			// If ground height is set, remove obstacles under it
			if(maxGroundHeight != 0.0f)
			{
				otherStuffIndices = rtabmap::util3d::passThrough(cloud, otherStuffIndices, "z", maxGroundHeight, std::numeric_limits<float>::max());
			}

			//Cluster remaining stuff (obstacles)
			if(otherStuffIndices->size())
			{
				std::vector<pcl::IndicesPtr> clusteredObstaclesSurfaces = util3d::extractClusters(
						cloud,
						otherStuffIndices,
						clusterRadius,
						minClusterSize);

				// merge indices
				obstacles = util3d::concatenate(clusteredObstaclesSurfaces);
			}
		}
	}
}



    
    

    return 0;
}

