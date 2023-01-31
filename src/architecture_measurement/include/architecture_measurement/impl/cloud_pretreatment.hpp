#pragma once
#ifndef _CLOUD_PRETREATMENT_H
#define _CLOUD_PRETREATMENT_H

#include "architecture_measurement/impl/fundamental.hpp"

#include <vector>
#include <memory>

#include "yaml-cpp/yaml.h"

#include "Eigen/Core"

#include "pcl/ModelCoefficients.h"
#include "pcl/point_types.h"
#include "pcl/common/distances.h"

namespace am
{
enum Quadrant{FIRST_QUADRANT = 1, SECOND_QUADRANT = 2, THIRD_QUADRANT = 3, FORTH_QUADRANT = 4};
enum Axis{AXIS_X = 1, AXIS_Y = 2, AXIS_Z = 3};

class CloudPretreatment
{
    private:
        class ParamsConfig;
        std::shared_ptr<ParamsConfig> _paramsConfig;

    public:
        CloudPretreatment();
        CloudPretreatment(const CloudPretreatment& cloud_pretreatment);
        ~CloudPretreatment();

        /**** common ****/
        void PointcloudTransform(pcl::PointCloud<pcl::PointXYZI>& src, pcl::PointCloud<pcl::PointXYZI>& dst, float theta = 0);
        int PointCloudTransformation(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, 
            const Eigen::Matrix3f& rotation, 
            const Eigen::Vector3f& translation);

        int GetMinMax3D(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
            pcl::PointXYZI& minPoint,
            pcl::PointXYZI& maxPoint);

        /**** filter ****/
        void StatisticalFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& res);
        void RadiusFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& res);
        void ConditionRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& res, const int& axis, const float& lower_limit, const float& upper_limit);
        void ConditionRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& res, const int& quadrant);
        int VoxelGridFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& res);
        int VoxelGridFilterMultiThreads(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& res);
        int DuplicateRemoval(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr dstPointCloudPtr);
        void ProjectPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& dst, const pcl::ModelCoefficients::Ptr& coeffs);
        void ShadowPointsFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& normals, pcl::PointCloud<pcl::PointXYZI>::Ptr& res);
        void MedianFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
        void PlaneClipper3D(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr& dst,      
            const Eigen::Vector4f& plane1, 
            const Eigen::Vector4f& plane2,
            const bool& nagative1 = false,            
            const bool& nagative2 = true);
        void PlaneClipper3D(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr& dst,      
            const Eigen::Vector4f& plane,
            const bool& nagative);

        /**** features ****/
        void NormalEstimation(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normal);
        void OrientedBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                                pcl::PointXYZI& minPoint, 
                                pcl::PointXYZI& maxPoint, 
                                pcl::PointXYZI& position,
                                Eigen::Matrix3f& rotationMatrix,
                                Eigen::Vector3f& majorVector,
                                Eigen::Vector3f& middleVector,
                                Eigen::Vector3f& minorVector,
                                Eigen::Vector3f& massCenter);
        void AxisAlignedBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                                    pcl::PointXYZI& minPoint, 
                                    pcl::PointXYZI& maxPoint, 
                                    Eigen::Vector3f& majorVector,
                                    Eigen::Vector3f& middleVector,
                                    Eigen::Vector3f& minorVector,
                                    Eigen::Vector3f& massCenter);
        void Boundary(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& normal, pcl::PointCloud<pcl::PointXYZI>::Ptr& res);
        
        /**** kdtree ****/
        // void Search

        /**** segmentation ****/
        void Extract(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::PointIndices::Ptr& indicesPtr, pcl::PointCloud<pcl::PointXYZI>::Ptr& dst, const bool& negative);      
        void Extract(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::PointIndices::Ptr& indicesPtr, pcl::PointCloud<pcl::PointXYZI>::Ptr& dst,const bool& negative, const bool& keepOrginized);
        pcl::ModelCoefficients Plane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
        pcl::ModelCoefficients Plane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers);
        void PalneSegmentation(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& normal, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cloudVector);
        int ParallelLine(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Eigen::Vector3f& axis, const int& number);
        int ParallelLine(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointIndices::Ptr& inliers, const Eigen::Vector3f& axis, const int& number);
        pcl::ModelCoefficients Line(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointIndices::Ptr& inliers);

        /**** surface ****/
        void ConcaveHull(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& dst, const int& demension);

        /**** visualization ****/
        void Viewer_PlaneClipper3D(pcl::PointCloud<pcl::PointXYZI>::Ptr& before, pcl::PointCloud<pcl::PointXYZI>::Ptr& after);
};

class CloudPretreatment::ParamsConfig
{
    private:
        /**filter**/
        float statisticalFilter_meanK;
        float statisticalFilter_stddevMulThresh;
        float radiusFilter_radius;
        int radiusFilter_minNeighborsInRadius;
        float voxelGridFilter_leafSize_x;
        float voxelGridFilter_leafSize_y;
        float voxelGridFilter_leafSize_z;
        float shadowPointsFilter_threshold;
        /**features**/
        int normalEstimation_numberOfThreads;
        int normalEstimation_kSearch;
        float boundary_radius;
        float boundary_angleThreshold;
        /**segment**/
        float plane_distanceThreshold;
        float parallelLine_epsAngle;
        float parallelLine_distanceThreshold;
        int planeSegmentation_minClusterSize;
        int planeSegmentation_maxClusterSize;
        int planeSegmentation_numberOfNeighbours;
        float planeSegmentation_smoothnessThreshold;
        float planeSegmentation_curvatureThreshold;
        /**surface**/
        float concaveHull_alpha;

        YAML::Node node;   

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

        /**filter**/
        inline float GetStatisticalFilter_meanK()
        {
            return statisticalFilter_meanK;
        }

        inline float GetStatisticalFilter_stddevMulThresh()
        {
            return statisticalFilter_stddevMulThresh;
        }

        inline float GetRadiusFilter_radius()
        {
            return radiusFilter_radius;
        }

        inline int GetRadiusFilter_minNeighborsInRadius()
        {
            return radiusFilter_minNeighborsInRadius;
        }

        inline float GetVoxelGridFilter_leafSize_x()
        {
            return voxelGridFilter_leafSize_x;
        }

        inline float GetVoxelGridFilter_leafSize_y()
        {
            return voxelGridFilter_leafSize_y;
        }

        inline float GetVoxelGridFilter_leafSize_z()
        {
            return voxelGridFilter_leafSize_z;
        }
        
        inline float GetShadowPointsFilter_threshold()
        {
            return shadowPointsFilter_threshold;
        }

        /**features**/
        inline int GetNormalEstimation_numberOfThreads()
        {
            return normalEstimation_numberOfThreads;
        }

        inline int GetNormalEstimation_kSearch()
        {
            return normalEstimation_kSearch;
        }

        inline float GetPlane_distanceThreshold()
        {
            return plane_distanceThreshold;
        }

        inline float GetParallelLine_epsAngle()
        {
            return parallelLine_epsAngle;
        }

        inline float GetBoundry_radius()
        {
            return boundary_radius;
        }

        inline float GetBoundry_angleThreshold()
        {
            return boundary_angleThreshold;
        }

        /**segment**/
        inline float GetParallelLine_distanceThreshold()
        {
            return parallelLine_distanceThreshold;
        }

        inline int GetPalneSegmentation_minClusterSize()
        {
            return planeSegmentation_minClusterSize;
        }

        inline int GetPalneSegmentation_maxClusterSize()
        {
            return planeSegmentation_maxClusterSize;
        }

        inline int GetPalneSegmentation_numberOfNeighbours()
        {
            return planeSegmentation_numberOfNeighbours;
        }

        inline float GetPalneSegmentation_smoothnessThreshold()
        {
            return planeSegmentation_smoothnessThreshold;
        }

        inline float GetPalneSegmentation_curvatureThreshold()
        {
            return planeSegmentation_curvatureThreshold;
        }

        /**surface**/
        inline float GetConcaveHull_alpha()
        {
            return concaveHull_alpha;
        }

};


}//namespace am

#endif //_CLOUD_PRETREATMENT_H