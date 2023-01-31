#ifndef DOOR_MEASURE_H
#define DOOR_MEASURE_H

#include "Utils.h"
#include "Global.h"
#include "ExtractOverlappingRegion.h"

class DoorRegistrate
{
public:
	DoorRegistrate() = default;
	~DoorRegistrate() = default;

	void InitTransformParams(std::shared_ptr<TransformParams> transformParamsPtr_);
	void InitPointCloudParams(std::shared_ptr<PointCloudParams> pointCloudParamsPtr_);

	void MergeClouds(std::vector<pcl::PointCloud<PointT> > vec_in_clouds,
		             std::vector<Eigen::Matrix4f> vec_transforms, 
		             std::vector<pcl::PointCloud<PointT> > &vec_out_clouds);

	void RunRegistrate(const std::vector<pcl::PointCloud<PointT> > &vec_in_clouds,
		                   const std::vector<Eigen::Matrix4f> &vec_transforms,
		                   pcl::PointCloud<PointT>& out_cloud);

private:
	std::shared_ptr<TransformParams> transformParamsPtr;
	std::shared_ptr<PointCloudParams> pointCloudParamsPtr;

	ExtractOverlappingRegion extractOverlappingPointCloud;

};

#endif