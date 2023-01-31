#ifndef FS_MEASURE_H
#define FS_MEASURE_H

#include <iostream>
#include <vector>

#include "Global.h"

#if defined(_MSC_VER)||defined(_WIN32)||defined(_WIN64)
#ifdef FSMEASURE_EXPORTS
#define FSMEASURE_API _declspec(dllexport)
#else
#define FSMEASURE_API _declspec(dllimport)
#endif
#else
#define FSMEASURE_API
#endif

class FSMEASURE_API FSMeasure   //这个类是个纯虚类, 不能被实例化
{
public:
	FSMeasure() = default;
	virtual ~FSMeasure() = default;

	virtual void Init(const std::string& configParams) = 0;	//=0表示这个函数没有实现，但是在继承类中必须要实现

	virtual void RunMergeClouds(std::vector<pcl::PointCloud<PointT> > vec_in_clouds,
								std::vector<Eigen::Matrix4f> vec_transforms,
								std::vector<pcl::PointCloud<PointT> >& vec_out_cloud) = 0;

	virtual void CalculatePerpendicularity(pcl::PointCloud<PointT>& in_cloud,
											const int& type,
											double& roll,
											double& pitch,
											double& yaw) = 0;

	virtual void RunDoorMeasure(const std::vector<pcl::PointCloud<PointT> >& vectorPointClouds,
		                        pcl::PointCloud<PointT>& outputPointCloud,
		                        float& holeWidth,
		                        float& holeHeight) = 0;

	virtual void RunWindowMeasure(const std::vector<pcl::PointCloud<PointT> >& vectorPointClouds,
		                          pcl::PointCloud<PointT>& outputPointCloud,
		                          float& holeWidth,
		                          float& holeHeight) = 0;

	static FSMeasure* Create();

	static int32_t Destory(FSMeasure* object);

};


#endif