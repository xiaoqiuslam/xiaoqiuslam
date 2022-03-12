#ifndef CORELIB_INCLUDE_RTABMAP_CORE_IMPL_OCCUPANCYGRID_HPP_
#define CORELIB_INCLUDE_RTABMAP_CORE_IMPL_OCCUPANCYGRID_HPP_

// #include <rtabmap/core/util3d_mapping.h>
// #include <rtabmap/core/util3d_transforms.h>
// #include <rtabmap/utilite/ULogger.h>

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

#include <vector>
#include <string>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>



using namespace cv;


pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloudIn,
		const pcl::IndicesPtr & indicesIn,
		const Eigen::Matrix4f transformation,
		const cv::Point3f & viewPoint,
		pcl::IndicesPtr & groundIndices,
		pcl::IndicesPtr & obstaclesIndices,
		pcl::IndicesPtr * flatObstacles)
{
	groundIndices.reset(new std::vector<int>);
	obstaclesIndices.reset(new std::vector<int>);
	if(flatObstacles)
	{
		flatObstacles->reset(new std::vector<int>);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::IndicesPtr indices(new std::vector<int>);

	bool preVoxelFiltering_;
	preVoxelFiltering_ = true;
	float cellSize_;

	if(preVoxelFiltering_)
	{
		// voxelize to grid cell size
		cloud = voxelizeImpl(cloudIn, indicesIn, cellSize_);

		indices->resize(cloud->size());
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			indices->at(i) = i;
		}
	}
	else
	{
		cloud = cloudIn;
		if(indicesIn->empty() && cloud->is_dense)
		{
			indices->resize(cloud->size());
			for(unsigned int i=0; i<indices->size(); ++i)
			{
				indices->at(i) = i;
			}
		}
		else
		{
			indices = indicesIn;
		}
	}
	{

	float footprintLength_;
	float footprintWidth_;
	float footprintHeight_;

	// filter footprint
	if(footprintLength_ > 0.0f || footprintWidth_ > 0.0f || footprintHeight_ > 0.0f)
	{
		indices = cropBoxImpl(
				cloud,
				indices,
				Eigen::Vector4f(
						footprintLength_>0.0f?-footprintLength_/2.0f:std::numeric_limits<int>::min(),
						footprintWidth_>0.0f&&footprintLength_>0.0f?-footprintWidth_/2.0f:std::numeric_limits<int>::min(),
						0,
						1),
				Eigen::Vector4f(
						footprintLength_>0.0f?footprintLength_/2.0f:std::numeric_limits<int>::max(),
						footprintWidth_>0.0f&&footprintLength_>0.0f?footprintWidth_/2.0f:std::numeric_limits<int>::max(),
						footprintHeight_>0.0f&&footprintLength_>0.0f&&footprintWidth_>0.0f?footprintHeight_:std::numeric_limits<int>::max(),
 {
						1),
				Transform::getIdentity(),
				true);
	}
 {

	// filter ground/obstacles zone
	if(minGroundHeight_ != 0.0f || maxObstacleHeight_ != 0.0f)
	{
		indices = passThroughImpl(cloud, indices, "z",
				minGroundHeight_!=0.0f?minGroundHeight_:std::numeric_limits<int>::min(),
				maxObstacleHeight_>0.0f?maxObstacleHeight_:std::numeric_limits<int>::max());
		UDEBUG("indices after max obstacles height filtering = %d", (int)indices->size());
	}

	if(indices->size())
	{
		if(normalsSegmentation_ && !groundIsObstacle_)
		{
			UDEBUG("normalKSearch=%d", normalKSearch_);
			UDEBUG("maxGroundAngle=%f", maxGroundAngle_);
			UDEBUG("Cluster radius=%f", clusterRadius_);
			UDEBUG("flatObstaclesDetected=%d", flatObstaclesDetected_?1:0);
			UDEBUG("maxGroundHeight=%f", maxGroundHeight_);
			UDEBUG("groundNormalsUp=%f", groundNormalsUp_);
			segmentObstaclesFromGround<PointT>(
					cloud,
					indices,
					groundIndices,
					obstaclesIndices,
					normalKSearch_,
					maxGroundAngle_,
					clusterRadius_,
					minClusterSize_,
					flatObstaclesDetected_,
					maxGroundHeight_,
					flatObstacles,
					Eigen::Vector4f(viewPoint.x, viewPoint.y, viewPoint.z+(projMapFrame_?pose.z():0), 1),
					groundNormalsUp_);
			UDEBUG("viewPoint=%f,%f,%f", viewPoint.x, viewPoint.y, viewPoint.z+(projMapFrame_?pose.z():0));
			//UWARN("Saving ground.pcd and obstacles.pcd");
			//pcl::io::savePCDFile("ground.pcd", *cloud, *groundIndices);
			//pcl::io::savePCDFile("obstacles.pcd", *cloud, *obstaclesIndices);
		}
		else
		{
			UDEBUG("");
			// passthrough filter
			groundIndices = rtabmap::util3d::passThrough(cloud, indices, "z",
					minGroundHeight_!=0.0f?minGroundHeight_:std::numeric_limits<int>::min(),
					maxGroundHeight_!=0.0f?maxGroundHeight_:std::numeric_limits<int>::max());

			pcl::IndicesPtr notObstacles = groundIndices;
			if(indices->size())
			{
				notObstacles = util3d::extractIndices(cloud, indices, true);
				notObstacles = util3d::concatenate(notObstacles, groundIndices);
			}
			obstaclesIndices = rtabmap::util3d::extractIndices(cloud, notObstacles, true);
		}

		UDEBUG("groundIndices=%d obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());

		// Do radius filtering after voxel filtering ( a lot faster)
		if(noiseFilteringRadius_ > 0.0 && noiseFilteringMinNeighbors_ > 0)
		{
			UDEBUG("Radius filtering (%ld ground %ld obstacles, radius=%f k=%d)",
					groundIndices->size(),
					obstaclesIndices->size()+(flatObstacles?(*flatObstacles)->size():0),
					noiseFilteringRadius_,
					noiseFilteringMinNeighbors_);
			if(groundIndices->size())
			{
				groundIndices = rtabmap::util3d::radiusFiltering(cloud, groundIndices, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			if(obstaclesIndices->size())
			{
				obstaclesIndices = rtabmap::util3d::radiusFiltering(cloud, obstaclesIndices, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			if(flatObstacles && (*flatObstacles)->size())
			{
				*flatObstacles = rtabmap::util3d::radiusFiltering(cloud, *flatObstacles, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			UDEBUG("Radius filtering end (%ld ground %ld obstacles)",
					groundIndices->size(),
					obstaclesIndices->size()+(flatObstacles?(*flatObstacles)->size():0));

			if(groundIndices->empty() && obstaclesIndices->empty())
			{
				UWARN("Cloud (with %d points) is empty after noise "
						"filtering. Occupancy grid cannot be "
						"created.",
						(int)cloud->size());

			}
		}
	}
	return cloud;



#endif /* CORELIB_INCLUDE_RTABMAP_CORE_IMPL_OCCUPANCYGRID_HPP_ */
