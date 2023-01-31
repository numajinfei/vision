#pragma once
#ifndef _CLOUD_PRETREATMENT_EX_H
#define _CLOUD_PRETREATMENT_EX_H

#include "architecture_measurement/impl/fundamental.hpp"

#include <vector>
#include <memory>
#include <iostream>
#include <exception>
#include <chrono>
#include <fstream>
#include <thread>
#include <ctime>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "yaml-cpp/yaml.h"

#include "pcl/ModelCoefficients.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/PointIndices.h"

#include "pcl/common/common.h"
#include "pcl/common/transforms.h"

#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/search/kdtree.h"

#include "pcl/features/normal_3d.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/features/moment_of_inertia_estimation.h"
#include "pcl/features/boundary.h"

#include "pcl/segmentation/region_growing.h"
#include "pcl/segmentation/sac_segmentation.h"

#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/filters/shadowpoints.h"
#include "pcl/filters/plane_clipper3D.h"

#include "pcl/sample_consensus/sac_model_plane.h"
#include "pcl/sample_consensus/sac_model_parallel_line.h"

#include "pcl/surface/concave_hull.h"

#include "pcl/visualization/pcl_visualizer.h"

/**
 * @brief architecture_measurement
 * 
 */
namespace am
{


/**
 * @brief point cloud pretreatment
 * 
 */
class CloudPretreatmentEx
{
    public:
        /**
         * @brief Construct a new Cloud Pretreatment object
         * 
         */
        CloudPretreatmentEx();

        /**
         * @brief Construct a new Cloud Pretreatment object
         * 
         * @param cloud_pretreatment a Cloud Pretreatment object
         */
        CloudPretreatmentEx(const CloudPretreatmentEx& cloud_pretreatment);

        /**
         * @brief Destroy the Cloud Pretreatment object
         * 
         */
        ~CloudPretreatmentEx();

        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param dstPointCloudPtr 
         * @param rotation 
         * @param translation 
         * @return int 
         */
        int PointCloudTransformation(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, 
            const Eigen::Matrix3f& rotation, 
            const Eigen::Vector3f& translation);

        int GetMinMax3D(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointXYZI& minPoint,
            pcl::PointXYZI& maxPoint);

        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param dstPointCloudPtr 
         * @param engineering 
         * @return int 
         */
        int StatisticalFilter(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, 
            const int& engineering = NORMAL);

        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param dstPointCloudPtr 
         * @param engineering 
         * @return int 
         */
        int RadiusFilter(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, 
            const int& engineering = NORMAL);

        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param dstPointCloudPtr 
         * @param axis 
         * @param lower_limit 
         * @param upper_limit 
         * @return int 
         */
        int ConditionRemoval(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, 
            const int& axis, 
            const float& lower_limit, 
            const float& upper_limit);
        
        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param dstPointCloudPtr 
         * @param quadrant 
         * @return int 
         */
        int ConditionRemoval(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, 
            const int& quadrant);

        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param dstPointCloudPtr 
         * @param engineering 
         * @return int 
         */
        int VoxelGridFilter(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, 
            const int& engineering = NORMAL);

        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param dstPointCloudPtr 
         * @return int 
         */
        int DuplicateRemoval(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr dstPointCloudPtr);

        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param dstPointCloudPtr 
         * @param coeffs 
         * @return int 
         */
        int ProjectPoints(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, 
            pcl::ModelCoefficients::Ptr& coeffs);

        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param pointCloudNormalPtr 
         * @param dstPointCloudPtr 
         * @return int 
         */
        int ShadowPointsFilter(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointCloud<pcl::Normal>::Ptr& pointCloudNormalPtr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr);
        
        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param dstPointCloudPtr 
         * @param plane1 
         * @param plane2 
         * @param nagative1 
         * @param nagative2 
         * @return int 
         */
        int PlaneClipper3D(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr,      
            const Eigen::Vector4f& plane1, 
            const Eigen::Vector4f& plane2,
            const bool& nagative1 = false,            
            const bool& nagative2 = true);

        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param dstPointCloudPtr 
         * @param plane 
         * @param nagative 
         * @return int 
         */
        int PlaneClipper3D(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr,      
            const Eigen::Vector4f& plane,
            const bool& nagative);

        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param pointCloudNromalPtr 
         * @param engineering 
         * @return int 
         */
        int NormalEstimation(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointCloud<pcl::Normal>::Ptr& pointCloudNromalPtr, 
            const int& engineering = NORMAL);

        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param minPoint 
         * @param maxPoint 
         * @param position 
         * @param rotationMatrix 
         * @param majorVector 
         * @param middleVector 
         * @param minorVector 
         * @param massCenter 
         * @return int 
         */
        int OrientedBoundingBox(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointXYZI& minPoint, 
            pcl::PointXYZI& maxPoint, 
            pcl::PointXYZI& position,
            Eigen::Matrix3f& rotationMatrix,
            Eigen::Vector3f& majorVector,
            Eigen::Vector3f& middleVector,
            Eigen::Vector3f& minorVector,
            Eigen::Vector3f& massCenter);
        
        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param minPoint 
         * @param maxPoint 
         * @param majorVector 
         * @param middleVector 
         * @param minorVector 
         * @param massCenter 
         * @return int 
         */
        int AxisAlignedBoundingBox(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointXYZI& minPoint, 
            pcl::PointXYZI& maxPoint, 
            Eigen::Vector3f& majorVector,
            Eigen::Vector3f& middleVector,
            Eigen::Vector3f& minorVector,
            Eigen::Vector3f& massCenter);
        
        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param pointCloudNormalPtr 
         * @param dstPointCloudPtr 
         * @return int 
         */
        int Boundary(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointCloud<pcl::Normal>::Ptr& pointCloudNormalPtr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr);
        
        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param indicesPtr 
         * @param dstPointCloudPtr 
         * @param negative 
         * @param keepOrginized 
         * @return int 
         */
        int Extract(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr,
            pcl::PointIndices::Ptr& indicesPtr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr,
            const bool& negative,
            const bool& keepOrginized = false);
        
        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @return pcl::ModelCoefficients 
         */
        pcl::ModelCoefficients Plane(pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr);
        
        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param inliersPtr 
         * @return pcl::ModelCoefficients 
         */
        pcl::ModelCoefficients Plane(pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, pcl::PointIndices::Ptr& inliersPtr);
        
        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param PointCloudNormalPtr 
         * @param pointCloudPtrVec 
         * @param engineering 
         * @return int 
         */
        int PlaneSegmentation(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointCloud<pcl::Normal>::Ptr& PointCloudNormalPtr, 
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& pointCloudPtrVec,
            const int& engineering = NORMAL);

        int PlaneSegmentation(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointCloud<pcl::Normal>::Ptr& PointCloudNormalPtr, 
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& pointCloudPtrVec,
            std::vector<pcl::PointIndices::Ptr>& pointIndicesPtrVec,
            const int& engineering = NORMAL);
        
        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param axis 
         * @param number 
         * @return pcl::ModelCoefficients 
         */
        pcl::ModelCoefficients ParallelLine(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            Eigen::Vector3f& axis ,
            const int& number);
        
        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param inliersPtr 
         * @param axis 
         * @param number 
         * @return pcl::ModelCoefficients 
         */
        pcl::ModelCoefficients ParallelLine(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointIndices::Ptr& inliersPtr, 
            const Eigen::Vector3f& axis,
            const int& number);

        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param inliersPtr 
         * @param axis 
         * @param number 
         * @param lineCoeffs 
         * @return int 
         */
        int ParallelLine(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointIndices::Ptr& inliersPtr, 
            const Eigen::Vector3f& axis,
            const int& number,
            pcl::ModelCoefficients& lineCoeffs);
        
        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @return pcl::ModelCoefficients 
         */
        pcl::ModelCoefficients Line(pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr);

        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param inliersPtr 
         * @return pcl::ModelCoefficients 
         */
        pcl::ModelCoefficients Line(pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, pcl::PointIndices::Ptr& inliersPtr);

        /**
         * @brief 
         * 
         * @param srcPointCloudPtr 
         * @param dstPointCloudPtr 
         * @param demension 
         * @return int 
         */
        int ConcaveHull(pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, const int& demension);

        /**
         * @brief 
         * 
         * @param PointCloudPtr1 
         * @param PointCloudPtr2 
         * @return int 
         */
        int Viewer_PlaneClipper3D(pcl::PointCloud<pcl::PointXYZI>::Ptr& PointCloudPtr1, pcl::PointCloud<pcl::PointXYZI>::Ptr& PointCloudPtr2);

    private:
        class ParamsConfig; /**< nested class for params config */
        std::shared_ptr<ParamsConfig> _paramsConfig; /**< ParamsConfig shared ptr */
};

/**
 * @brief class for parameters config
 * 
 */
class CloudPretreatmentEx::ParamsConfig
{
    public:
        /**
         * @brief Construct a new Params Config object
         * 
         */
        ParamsConfig();

        /**
         * @brief Construct a new Params Config object
         * 
         * @param fileName param file in yaml format
         */
        ParamsConfig(const std::string& fileName);

        /**
         * @brief Destroy the Params Config object
         * 
         */
        ~ParamsConfig();
        
        /**
         * @brief load param file
         * 
         * @param fileName param file in yaml format
         */
        void Load(const std::string& fileName);

        /**
         * @brief parse yaml file
         * 
         */
        void Parse();

        /**
         * @brief print content of param file
         * 
         */
        void PrintParam();

    public:
        YAML::Node node;  

        float statisticalFilter_meanK;
        float statisticalFilter_stddevMulThresh;

        float radiusFilter_radius;
        int radiusFilter_minNeighborsInRadius;

        float voxelGridFilter_leafSize_x;
        float voxelGridFilter_leafSize_y;
        float voxelGridFilter_leafSize_z;

        float shadowPointsFilter_threshold;

        int normalEstimation_numberOfThreads;
        int normalEstimation_kSearch;

        float boundary_radius;
        float boundary_angleThreshold;

        float plane_distanceThreshold;

        float parallelLine_epsAngle;
        float parallelLine_distanceThreshold;

        float line_distanceThreshold;

        int planeSegmentation_minClusterSize;
        int planeSegmentation_maxClusterSize;
        int planeSegmentation_numberOfNeighbours;
        float planeSegmentation_smoothnessThreshold;
        float planeSegmentation_curvatureThreshold;

        float concaveHull_alpha;

        class MasonryWall
        {
            public:
                int normalEstimation_numberOfThreads;
                int normalEstimation_kSearch; 

                int planeSegmentation_minClusterSize;
                int planeSegmentation_maxClusterSize;
                int planeSegmentation_numberOfNeighbours;
                float planeSegmentation_smoothnessThreshold;
                float planeSegmentation_curvatureThreshold;
        } masonryWall;

};

}//namespace am

#endif //_CLOUD_PRETREATMENT_H