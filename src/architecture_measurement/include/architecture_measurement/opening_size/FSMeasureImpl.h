#ifndef FS_MEASURE_IMPL_H
#define FS_MEASURE_IMPL_H

#include "FSMeasure.h"

#include "DoorRegistrate.h"
#include "Perpendicularity.h"
#include "ParamsConfig.h"

#include "DoorWindowMeasure.h"

class FSMeasureImpl:public FSMeasure
{
public:
	FSMeasureImpl();
	~FSMeasureImpl() = default;

	void Init(const std::string &configParams) override;

	void RunMergeClouds(std::vector<pcl::PointCloud<PointT> > vec_in_clouds,
		                std::vector<Eigen::Matrix4f> vec_transforms, 
		                std::vector<pcl::PointCloud<PointT> >& vec_out_cloud) override;

	void CalculatePerpendicularity(pcl::PointCloud<PointT>& in_cloud, 
		                           const int& type,
		                           double& roll, 
		                           double& pitch, 
		                           double& yaw) override;

	void RunDoorMeasure(const std::vector<pcl::PointCloud<PointT> > &vectorPointClouds,
		                pcl::PointCloud<PointT> &outputPointCloud,
		                float& holeWidth,
		                float& holeHeight) override;

	void RunWindowMeasure(const std::vector<pcl::PointCloud<PointT> > &vectorPointClouds,
		                  pcl::PointCloud<PointT> &outputPointCloud,
		                  float& holeWidth,
		                  float& holeHeight) override;

private:
	DoorRegistrate doorRegistrate;
	Perpendicularity perpendicularity;
	
	DoorWindowMeasure doorWindowHoleMeasure;

	std::shared_ptr<TransformParams> transformParamsPtr; 
	std::shared_ptr<PointCloudParams> pointCloudParamsPtr;

};

#endif