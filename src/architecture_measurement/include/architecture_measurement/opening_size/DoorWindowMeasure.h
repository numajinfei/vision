#ifndef POINTCLOUDPOSTPROCESS_H
#define POINTCLOUDPOSTPROCESS_H

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
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

#include <cfloat>
#include <climits>

#include "Utils.h"

struct Point2LineDistance
{
	float distance = 0.0f;
	int lineIndex = -1;
};

class DoorWindowMeasure
{
public:
	DoorWindowMeasure() = default;
	~DoorWindowMeasure() = default;
	void InitPointCloudParams(std::shared_ptr<PointCloudParams> pointCloudParamsPtr_);

	void RunDoorWindowHoleMeasure(const pcl::PointCloud<PointT> &inputPointCloud, float &holeWidth, float &holeHeight);


private:
	void RansacPlaneSegmentation(const pcl::PointCloud<PointT> &inputPointCloud, 
		                         const float &planeSegmentPointThreshold,
		                         const int &planeSegmentMaxIterations,
		                         const float &planeDistanceThreshold,
		                         std::vector<pcl::PointCloud<PointT>::Ptr> &pointcloudPlane3d,
		                         std::vector<Eigen::Vector3f> &planeNormals,
		                         std::vector<std::vector<float> > &planeCoefficientsVector,
		                         int &maxPointNumIndex);

	void ExtractPointCloudBoundary(const pcl::PointCloud<PointT> &planePointCloudROI,
		                           pcl::PointCloud<PointT>::Ptr  &pointCloudROIBoundary,
		                           PointT &pointCloudROICentert);

	void RansacLineSegmentation(const pcl::PointCloud<PointT>::Ptr  &pointCloudROIBoundary,
		                        const float &lineSegmentPointThreshold,
		                        const int &lineSegmentMaxIterations,
		                        const float &lineSegmentDistanceThreshold,
		                        std::vector<pcl::PointCloud<PointT>::Ptr> &pointCloudLine3d,
		                        std::vector<Eigen::Vector3f> &lineCoefficients,
	                            std::vector<std::vector<float> > &lineCoefficientsVector,
		                        int &lineCount);

	void CalculateLineIndexNearestDistance(const PointT &pointCloudROICenter,
		                                   const std::vector<int> &lineHorizontalUpIndex,
		                                   const std::vector<int> &lineHorizontalDownIndex,
	                                       const std::vector<int> &lineVerticalLeftIndex,
	                                       const std::vector<int> &lineVerticalRightIndex,
		                                   std::vector<pcl::PointCloud<PointT>::Ptr> &pointCloudLine3d,
		                                   std::vector<Point2LineDistance> &pointHorizontalUpDistances,
		                                   std::vector<Point2LineDistance> &pointHorizontalDownDistances,
		                                   std::vector<Point2LineDistance> &pointVerticalLeftDistances,
		                                   std::vector<Point2LineDistance> &pointVerticalRightDistances);
	
	void linePreClassification(const std::vector<Eigen::Vector3f> &planeNormals,
		                       const std::vector<Eigen::Vector3f> &lineCoefficients,
		                       const int &sideWallPointNumIndex,
		                       const int &groundPointNumIndex,
	                           std::vector<int> &lineHorizontalIndex,
	                           std::vector<int> &lineVerticalIndex);
	
	
	
	void lineClassification(const PointT &pointCloudROICenter,
		                    const std::vector<int> &lineHorizontalIndex,
	                        const std::vector<int> &lineVerticalIndex,
		                    std::vector<pcl::PointCloud<PointT>::Ptr> &pointCloudLine3d,
	                        std::vector<int> &lineHorizontalUpIndex,
	                        std::vector<int> &lineHorizontalDownIndex,
	                        std::vector<int> &lineVerticalLeftIndex,
	                        std::vector<int> &lineVerticalRightIndex);

	void CalcGroundPointNumIndex(const std::vector<Eigen::Vector3f> &planeNormals,
		                         const int &maxPointNumIndex, 
		                         int &groundPointNumIndex, 
		                         std::vector<int> &planeParallelground);

	float CalculateTargetHeight(std::vector<pcl::PointCloud<PointT>::Ptr> &pointcloudPlane3d,
		                        const std::vector<std::vector<float> > &planeCoefficientsVector,
		                        const std::vector<int> &planeParallelground,
		                        const int &groundPointNumIndex);

	float CalculateTargetWidth(std::vector<pcl::PointCloud<PointT>::Ptr> &pointCloudLine3d,
		                       const std::vector<std::vector<float> > &planeCoefficientsVector,
		                       const std::vector<int> &verticalIndexLines,
		                       const int &holeSidePlanePointNumIndex);

	void CalcSideWallPlaneIndex(const PointT &pointCloudROICenter,
		                        std::vector<pcl::PointCloud<PointT>::Ptr> &pointcloudPlane3d,
		                        const std::vector<Eigen::Vector3f> &planeNormals,
		                        const int &groundPointNumIndex,
		                        const int &maxPointNumIndex,
		                        int &sideWallPointNumIndex, // 基准侧面（最大）索引
		                        std::vector<int>  &sideWallPlaneIndex);

	void CalcLine2SidePlaneDistance(std::vector<pcl::PointCloud<PointT>::Ptr> &pointCloudLine3d, 
		                            const std::vector<std::vector<float> > &planeCoefficientsVector,
		                            const std::vector<int>  &sideWallPlaneIndex,
		                            const std::vector<int> &verticalIndexLines,
		                            int &holeSidePlanePointNumIndex);
private:
	//std::shared_ptr<TransformParams> transformParamsPtr;
	std::shared_ptr<PointCloudParams> pointCloudParamsPtr;

};





#endif