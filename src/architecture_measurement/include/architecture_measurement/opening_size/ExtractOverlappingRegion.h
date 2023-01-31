#ifndef EXTRACTOVERLAPPINGREGION_H
#define EXTRACTOVERLAPPINGREGION_H

#include <iostream>
#include <vector>
//#include <ctime>
//#include <boost/thread/thread.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/console/parse.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/boundary.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include "pcl/point_cloud.h"
#include <pcl/features/boundary.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>   // 随机参数估计方法
#include <pcl/sample_consensus/model_types.h>    // 模型定义
#include <pcl/segmentation/sac_segmentation.h>   // 采样一致性分割
#include <pcl/filters/extract_indices.h>
#include "pcl/segmentation/region_growing.h"
#include "pcl/features/normal_3d_omp.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/filters/radius_outlier_removal.h"
#include <cfloat>
#include <climits>
#include "Utils.h"

class ExtractOverlappingRegion
{
public:
	ExtractOverlappingRegion() = default;
	~ExtractOverlappingRegion() = default;

	//void Init();
	void InitTransformParams(std::shared_ptr<TransformParams> transformParamsPtr_);
	void InitPointCloudParams(std::shared_ptr<PointCloudParams> pointCloudParamsPtr_);

	void RunExtractOverlappingRegion(const std::vector<pcl::PointCloud<PointT> > &vectorPointClouds,
		                             const std::vector<Eigen::Matrix4f> &correspondTransforms, 
		                             std::vector<pcl::PointCloud<PointT> > &overlapCameraPointCloud,
		                             std::vector<pcl::PointCloud<PointT> > &correspondCameraOverlapPointCloud);

private:
	Eigen::Vector2f reprojectCameraPoint2ImagePlane(const cv::Mat &cameraMatrix,
		                                            const cv::Mat &distortParams, 
		                                            const PointT &cameraCoordinatePoint);


	bool ValidPixel(const Eigen::Vector2f &imagePixelResult,
		            const float &min_x,
		            const float &max_x,
		            const float &min_y,
		            const float &max_y);
private:
	std::shared_ptr<TransformParams> transformParamsPtr;
	std::shared_ptr<PointCloudParams> pointCloudParamsPtr;
	
	////cv::Mat cameraMatrix;
	////cv::Mat distortMatrix;
	////float imageSizeWidth;
	////float imageSizeHeight;

};





#endif