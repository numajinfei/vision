#pragma once
#ifndef GLOBAL_H
#define GLOBAL_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/ply_io.h>

typedef pcl::PointXYZI PointT;

enum Axis 
{ 
	AXIS_X = 1, 
	AXIS_Y = 2, 
	AXIS_Z = 3 
};

enum HResult
{
    SS_OK = 0,	        //操作成功
    SS_FALSE = 1,	    //函数成功执行完成，但返回时出现错误
    EE_INVALIDARG = 2,	//一个或多个参数无效
    EE_POINTER = 3,	    //无效指针
    EE_FAIL = 4,	    // 未指定的失败
    EE_OUTOFMEMORY = 5,	//内存申请错误
    EE_UNEXPECTED = 6	//未知的异常
};

struct TransformParams
{
	Eigen::Vector2f imageSize;
	cv::Mat cameraIntrinsicMatrix;
	cv::Mat cameraDistortMatrix;

    Eigen::Matrix4f up30_transform;
    Eigen::Matrix4f up20_transform;
    Eigen::Matrix4f up10_transform;
    Eigen::Matrix4f down10_transform;
    Eigen::Matrix4f down20_transform;
    Eigen::Matrix4f down30_transform;

};


struct PointCloudParams
{
    /**filter**/
    float voxelGridFilter_leafSize;
    double radiusFilter_radius;
    int radiusFilter_minNeighborsInRadius;
    int statisticalFilter_meanK;
    double statisticalFilter_stddevMulThresh;
    /**registration**/
    double icp_distanceThreshold;
    int icp_maxIterations;
    double icp_transformationEpsilon;
    double icp_euclideanFitnessEpsilon;
	/**PlaneSegmentation**/
	float plane_segmentation_PointThreshold;
	int plane_segmentation_MaxIterations;
	float plane_segmentation_DistanceThreshold;
	/*LineSegmentation**/
	float line_segmentation_PointThreshold;
	int line_segmentation_MaxIterations;
	float line_segmentation_DistanceThreshold;

};

#endif