/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef UTIL3D_MAPPING_HPP_
#define UTIL3D_MAPPING_HPP_

#include <boost/make_shared.hpp>

#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>

namespace rtabmap{
namespace util3d{

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr projectCloudOnXYPlane(
		const typename pcl::PointCloud<PointT> & cloud)
{
	typename pcl::PointCloud<PointT>::Ptr output = boost::make_shared<pcl::PointCloud<PointT>>();
	*output = cloud;
	for(unsigned int i=0; i<output->size(); ++i)
	{
		output->at(i).z = 0;
	}
	return output;
}

template<typename PointT>
void segmentObstaclesFromGround(
		const typename pcl::PointCloud<PointT>::Ptr& cloud,
		const typename pcl::IndicesPtr& indices,
		pcl::IndicesPtr& ground,
		pcl::IndicesPtr& obstacles,
		pcl::IndicesPtr& underground,
		int normalKSearch,
		float groundNormalAngle,
		float clusterRadius,
		int minClusterSize,
		bool segmentFlatObstacles,
		float maxGroundHeight,
		pcl::IndicesPtr* flatObstacles,
		const Eigen::Vector4f& viewPoint)
{
	Eigen::Vector4f minGround;
	Eigen::Vector4f maxGround;

	ground = boost::make_shared<std::vector<int>>();
	obstacles = boost::make_shared<std::vector<int>>();
	underground = boost::make_shared<std::vector<int>>();
	if (flatObstacles) {
		*flatObstacles = boost::make_shared<std::vector<int>>();
	}

	if (cloud->size()) {
		// Find the ground
		pcl::IndicesPtr flatSurfaces = normalFiltering(
				cloud,
				indices,
				groundNormalAngle,
				Eigen::Vector4f(0,0,1,0),
				normalKSearch,
				viewPoint);
		UINFO("AVIDBOTS: segmentFlatObstacles=%d flatSurfaces->size=%llu", segmentFlatObstacles?1:0, flatSurfaces->size());

		if (segmentFlatObstacles && flatSurfaces->size()) {
			int biggestFlatSurfaceIndex;
			std::vector<pcl::IndicesPtr> clusteredFlatSurfaces = extractClusters(
					cloud,
					flatSurfaces,
					clusterRadius,
					minClusterSize,
					std::numeric_limits<int>::max(),
					&biggestFlatSurfaceIndex);
			UINFO("AVIDBOTS: biggestFlatSurfaceIndex=%d", biggestFlatSurfaceIndex);
			UINFO("AVIDBOTS: clusteredFlatSurfaces.size=%llu", clusteredFlatSurfaces.size());

			// cluster all surfaces for which the centroid is in the Z-range of the bigger surface
			if (clusteredFlatSurfaces.size()) {
				ground = clusteredFlatSurfaces.at(biggestFlatSurfaceIndex);
				Eigen::Vector4f& min = minGround;
				Eigen::Vector4f& max = maxGround;
				pcl::getMinMax3D(*cloud, *clusteredFlatSurfaces.at(biggestFlatSurfaceIndex), min, max);

				if (maxGroundHeight == 0.0f || min[2] < maxGroundHeight) {
					for(unsigned int i=0; i<clusteredFlatSurfaces.size(); ++i) {
						// XXX: AVIDBOTS BEGIN
						if (false) {
							Eigen::Vector4f centroid(0,0,0,1);
							pcl::compute3DCentroid(*cloud, *clusteredFlatSurfaces.at(i), centroid);

							Eigen::Vector4f mn, mx;
							pcl::getMinMax3D(*cloud, *clusteredFlatSurfaces.at(i), mn, mx);
							// if (mn[2] <= -0.08) {
								std::ostringstream oss;
								oss << "mn=" << mn.transpose() << " mx=" << mx.transpose() << " centroid=" << centroid.transpose();
								UINFO("AVIDBOTS: cliff segment: %s, size=%u",
											oss.str().c_str(), clusteredFlatSurfaces.at(i)->size());
							// }
						}
						// XXX: AVIDBOTS END

						if ((int)i != biggestFlatSurfaceIndex)
						{
							Eigen::Vector4f centroid(0, 0, 0, 1);
							pcl::compute3DCentroid(*cloud, *clusteredFlatSurfaces.at(i), centroid);

							if (maxGroundHeight==0.0f || centroid[2] <= maxGroundHeight || centroid[2] <= max[2]) { // epsilon
								UINFO("AVIDBOTS: i=%u: concatenate ground", i);
								ground = util3d::concatenate(ground, clusteredFlatSurfaces.at(i));
							} else if (flatObstacles) {
								UINFO("AVIDBOTS: i=%u: concatenate obstacles", i);
								*flatObstacles = util3d::concatenate(*flatObstacles, clusteredFlatSurfaces.at(i));
							}
						}
					}
				} else {
					// reject ground!
					ground = boost::make_shared<std::vector<int>>();
					if (flatObstacles) {
						*flatObstacles = flatSurfaces;
					}
				}
			}
		} else {
			ground = flatSurfaces;
		}

		if (ground->size() != cloud->size()) {
			// Remove ground
			pcl::IndicesPtr notObstacles = ground;
			if (indices->size()) {
				notObstacles = util3d::extractIndices(cloud, indices, true);
				notObstacles = util3d::concatenate(notObstacles, ground);
			}
			pcl::IndicesPtr otherStuffIndices = util3d::extractIndices(cloud, notObstacles, true);
			pcl::IndicesPtr undergroundStuffIndices;
			// If ground height is set, remove obstacles under it
			if (maxGroundHeight != 0.0f) {
				undergroundStuffIndices = rtabmap::util3d::passThrough(cloud, otherStuffIndices, "z", std::numeric_limits<float>::lowest(), maxGroundHeight);
				otherStuffIndices = rtabmap::util3d::passThrough(cloud, otherStuffIndices, "z", maxGroundHeight, std::numeric_limits<float>::max());
			} else {
				undergroundStuffIndices = rtabmap::util3d::passThrough(cloud, otherStuffIndices, "z", std::numeric_limits<float>::lowest(), minGround[2]);
				otherStuffIndices = rtabmap::util3d::passThrough(cloud, otherStuffIndices, "z", maxGround[2], std::numeric_limits<float>::max());
			}

			// otherStuffIndices = rtabmap::util3d::passThrough(cloud, otherStuffIndices, "z", std::numeric_limits<float>::lowest(), maxGroundHeight, true);

			// Cluster remaining stuff (obstacles)
			if (otherStuffIndices->size()) {
				std::vector<pcl::IndicesPtr> clusteredObstaclesSurfaces = util3d::extractClusters(
						cloud,
						otherStuffIndices,
						clusterRadius,
						minClusterSize);

				// merge indices
				obstacles = util3d::concatenate(clusteredObstaclesSurfaces);
			}

			// Cluster remaining stuff (obstacles)
			if (undergroundStuffIndices) {
				std::vector<pcl::IndicesPtr> clusteredUndergroundSurfaces = util3d::extractClusters(
						cloud,
						undergroundStuffIndices,
						clusterRadius,
						minClusterSize);

				// merge indices
				underground = util3d::concatenate(clusteredUndergroundSurfaces);
			}
		}
	}
}

template<typename PointT>
void segmentObstaclesFromGround(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		pcl::IndicesPtr & ground,
		pcl::IndicesPtr & obstacles,
		pcl::IndicesPtr & underground,
		int normalKSearch,
		float groundNormalAngle,
		float clusterRadius,
		int minClusterSize,
		bool segmentFlatObstacles,
		float maxGroundHeight,
		pcl::IndicesPtr * flatObstacles,
		const Eigen::Vector4f & viewPoint)
{
	pcl::IndicesPtr indices = boost::make_shared<std::vector<int>>();
	segmentObstaclesFromGround<PointT>(
			cloud,
			indices,
			ground,
			obstacles,
			underground,
			normalKSearch,
			groundNormalAngle,
			clusterRadius,
			minClusterSize,
			segmentFlatObstacles,
			maxGroundHeight,
			flatObstacles,
			viewPoint);
}

template<typename PointT>
void occupancy2DFromGroundObstacles(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & groundIndices,
		const pcl::IndicesPtr & obstaclesIndices,
		const pcl::IndicesPtr & undergroundIndices,
		cv::Mat & ground,
		cv::Mat & obstacles,
		cv::Mat & underground,
		float cellSize)
{
	typename pcl::PointCloud<PointT>::Ptr groundCloud = boost::make_shared<pcl::PointCloud<PointT>>();
	typename pcl::PointCloud<PointT>::Ptr obstaclesCloud = boost::make_shared<pcl::PointCloud<PointT>>();
	typename pcl::PointCloud<PointT>::Ptr undergroundCloud = boost::make_shared<pcl::PointCloud<PointT>>();

	if(groundIndices->size())
	{
		pcl::copyPointCloud(*cloud, *groundIndices, *groundCloud);
	}

	if(obstaclesIndices->size())
	{
		pcl::copyPointCloud(*cloud, *obstaclesIndices, *obstaclesCloud);
	}

	if(undergroundIndices->size())
	{
		pcl::copyPointCloud(*cloud, *undergroundIndices, *undergroundCloud);
	}

	occupancy2DFromGroundObstacles<PointT>(
			groundCloud,
			obstaclesCloud,
			undergroundCloud,
			ground,
			obstacles,
			underground,
			cellSize);
}

template<typename PointT>
void occupancy2DFromGroundObstacles(
		const typename pcl::PointCloud<PointT>::Ptr & groundCloud,
		const typename pcl::PointCloud<PointT>::Ptr & obstaclesCloud,
		const typename pcl::PointCloud<PointT>::Ptr & undergroundCloud,
		cv::Mat & ground,
		cv::Mat & obstacles,
		cv::Mat & underground,
		float cellSize)
{
	ground = cv::Mat();
	if(groundCloud->size())
	{
		//project on XY plane
		typename pcl::PointCloud<PointT>::Ptr groundCloudProjected;
		groundCloudProjected = util3d::projectCloudOnXYPlane(*groundCloud);
		//voxelize to grid cell size
		groundCloudProjected = util3d::voxelize(groundCloudProjected, cellSize);

		ground = cv::Mat(1, (int)groundCloudProjected->size(), CV_32FC2);
		for(unsigned int i=0;i<groundCloudProjected->size(); ++i)
		{
			cv::Vec2f * ptr = ground.ptr<cv::Vec2f>();
			ptr[i][0] = groundCloudProjected->at(i).x;
			ptr[i][1] = groundCloudProjected->at(i).y;
		}
	}

	obstacles = cv::Mat();
	if(obstaclesCloud->size())
	{
		//project on XY plane
		typename pcl::PointCloud<PointT>::Ptr obstaclesCloudProjected;
		obstaclesCloudProjected = util3d::projectCloudOnXYPlane(*obstaclesCloud);
		//voxelize to grid cell size
		obstaclesCloudProjected = util3d::voxelize(obstaclesCloudProjected, cellSize);

		obstacles = cv::Mat(1, (int)obstaclesCloudProjected->size(), CV_32FC2);
		for(unsigned int i=0; i<obstaclesCloudProjected->size(); ++i)
		{
			cv::Vec2f * ptr = obstacles.ptr<cv::Vec2f>();
			ptr[i][0] = obstaclesCloudProjected->at(i).x;
			ptr[i][1] = obstaclesCloudProjected->at(i).y;
		}
	}
}

template<typename PointT>
void occupancy2DFromCloud3D(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		cv::Mat & ground,
		cv::Mat & obstacles,
		cv::Mat & underground,
		float cellSize,
		float groundNormalAngle,
		int minClusterSize,
		bool segmentFlatObstacles,
		float maxGroundHeight)
{
	if(cloud->size() == 0)
	{
		return;
	}
	pcl::IndicesPtr groundIndices, obstaclesIndices, undergroundIndices;

	segmentObstaclesFromGround<PointT>(
			cloud,
			indices,
			groundIndices,
			obstaclesIndices,
			undergroundIndices,
			20,
			groundNormalAngle,
			cellSize*2.0f,
			minClusterSize,
			segmentFlatObstacles,
			maxGroundHeight);

	occupancy2DFromGroundObstacles<PointT>(
			cloud,
			groundIndices,
			obstaclesIndices,
			undergroundIndices,
			ground,
			obstacles,
			underground,
			cellSize);
}

template<typename PointT>
void occupancy2DFromCloud3D(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		cv::Mat & ground,
		cv::Mat & obstacles,
		cv::Mat & underground,
		float cellSize,
		float groundNormalAngle,
		int minClusterSize,
		bool segmentFlatObstacles,
		float maxGroundHeight)
{
	pcl::IndicesPtr indices = boost::make_shared<std::vector<int>>();
	occupancy2DFromCloud3D<PointT>(cloud, indices, ground, obstacles, underground, cellSize, groundNormalAngle, minClusterSize, segmentFlatObstacles, maxGroundHeight);
}

}
}

#endif /* UTIL3D_MAPPING_HPP_ */
