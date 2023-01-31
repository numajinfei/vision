#define _CRT_SECURE_NO_WARNINGS  //允许不安全函数及过时的函数,windows系统下

#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>
//#include <pcl/point_types.h>
#include <pcl/point_traits.h>
//#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/registration/icp.h>

#include <iostream>
#include <stdio.h>

#include "Global.h"

//定义点云格式
//typedef pcl::PointXYZI PointT;

//enum Axis 
//{ 
//	AXIS_X = 1, 
//	AXIS_Y = 2, 
//	AXIS_Z = 3 
//};

class Utils
{
public:

	static void CalcNormalDirect(const float &rollAngle,
		                         const float &pitchAngle,
		                         cv::Mat &outputNormal);

	static float CalcAngleBetweenTwoVector(cv::Mat targetVector, cv::Mat sourceVector);

	static void VoxelGridFilter(const pcl::PointCloud<PointT>& in_cloud, pcl::PointCloud<PointT>& out_cloud, const float& down_sample_size);
	
	static void RadiusFilter(const pcl::PointCloud<PointT>& in_cloud, pcl::PointCloud<PointT>& out_cloud, 
								const double& radius, const int& min_pts);
	
	static void StatisticalFilter(const pcl::PointCloud<PointT>& in_cloud, pcl::PointCloud<PointT>& out_cloud, 
									const int& meanK, const double& stddevMulThresh);
	
	static void ConditionRemoval(const pcl::PointCloud<PointT>& in_cloud, pcl::PointCloud<PointT>& out_cloud,
									const int& axis, const float& lower_limit, const float& upper_limit);
	
	static void ProjectFilter(const pcl::PointCloud<PointT>& in_cloud, pcl::PointCloud<PointT>& out_cloud,
								const pcl::ModelCoefficients::Ptr& coeffs);
	
	static void ShadowPointsFilter(const pcl::PointCloud<PointT>& in_cloud, pcl::PointCloud<PointT>& out_cloud,
									const pcl::PointCloud<pcl::Normal>::Ptr& normals, float threshold);
	
	static void MedianFilter(pcl::PointCloud<PointT>& in_cloud, int size);

	static int* RandRGB();

	static void VecCloudsViewer(std::vector<pcl::PointCloud<PointT> >& vec_clouds);

	static double Degree2Radian(const double& degree);
	static double Radian2Degree(const double& radian);

    static bool SortPointCloudZL(PointT& point1, PointT& point2);
    static bool SortPointCloudZG(PointT& point1, PointT& point2);

	static Eigen::Matrix3d GetMatrixFromRPY(const double& roll, const double& pitch, const double& yaw);

	static void BoundaryExtraction_normalEstimation(pcl::PointCloud<PointT>& in_cloud,
		pcl::PointCloud<PointT>& cloud_boundary, pcl::PointCloud<PointT>& cloud_noBound);

    static void Boundary(const pcl::PointCloud<PointT>& in_cloud, const pcl::PointCloud<pcl::Normal>& normal,
        pcl::PointCloud<PointT>& out_cloud, const double& radius, float& angleThreshold);

	static void NormalEstimation(const pcl::PointCloud<PointT>& in_cloud, pcl::PointCloud<pcl::Normal>& normal,
		const unsigned int& numberOfThreads, const int& kSearch);

	static void Extract(const pcl::PointCloud<PointT>& in_cloud, const pcl::PointIndices::Ptr& indicesPtr,
		pcl::PointCloud<PointT>& out_cloud, const bool& negative);

	static void PlaneSegmentation(const pcl::PointCloud<PointT>& in_cloud,
		const pcl::PointCloud<pcl::Normal>::Ptr& normal, std::vector<pcl::PointCloud<PointT>::Ptr>& cloudVector,
		const int& minClusterSize, const int& MaxClusterSize, const int& numberOfNeighbours,
		const float& smoothnessThreshold, const float& curvatureThreshold);

	static pcl::ModelCoefficients Plane(const pcl::PointCloud<PointT>& in_cloud, const double& distanceThreshold);

    static int ParallelLine(const pcl::PointCloud<PointT>& in_cloud, const Eigen::Vector3f& axis,
        const double& epsAngle, const double distanceThreshold, const int& number);

    static pcl::ModelCoefficients Line(pcl::PointCloud<PointT>& in_cloud, pcl::PointIndices& inliers, const double distanceThreshold);

    static void PointcloudTransform(pcl::PointCloud<PointT>& src, pcl::PointCloud<PointT>& dst, float theta);

    static int Write(const pcl::PointCloud<PointT>& out_cloud, const std::string& fileName);

    static int Read(const std::string& fileName, pcl::PointCloud<PointT>& in_cloud);

	static void RefinedRegistrate(const pcl::PointCloud<PointT> &inputTargetPointCloud,
	                          	  const pcl::PointCloud<PointT> &inputSourcePointCloud,
                              	  const double &distance_threshold,
                              	  const int &max_iterations,
                              	  const double &transformation_epsilon,
                              	  const double &euclidean_fitness_psilon,
	                          	  Eigen::Matrix4f &outputTransformMatrix);

	static void GetMatrixFromTxt(const std::string& fileName, Eigen::Matrix4f& mat);

	// ZLT于2022.10.26
	static bool Point2LineDistance(const std::vector<float> &lineCoefficient, const PointT &point3D, float &point2LineDistance);

	static float PointToPointDistance(const PointT &point1, const PointT &point2);

	static void LoadPoseTransforms(const std::string &file_path, Eigen::Matrix4f &poseTransforms);

	static void ReadPoseTransforms(const std::string &fileName, Eigen::Matrix4f &poseTransforms);


	static float PointToPlaneDistance(const PointT &inputPoint,
		                              const std::vector<float> planeParams);

	static void Search2DMaxMinLoc(const std::vector<Eigen::Vector2f> &imageValidPixel,
		                          float &min_x,
		                          float &max_x,
		                          float &min_y,
		                          float &max_y);

	static void RefinedRegistrateWithPoint2Plane(const pcl::PointCloud<pcl::PointNormal> &inputTargetPointCloud,
		                                         const pcl::PointCloud<pcl::PointNormal> &inputSourcePointCloud,
		                                         Eigen::Matrix4f &outputTransformMatrix);


private:
    static const unsigned int CAPACITY = 3200000;

};


#endif