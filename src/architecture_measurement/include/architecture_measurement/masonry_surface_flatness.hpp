#pragma once
#ifndef _MASONRY_SURFACE_FLATNESS_H
#define _MASONRY_SURFACE_FLATNESS_H

#include "architecture_measurement/impl/inclinometer.hpp"
#include "architecture_measurement/impl/point_cloud_gui.hpp"
#include "architecture_measurement/impl/fundamental.hpp"
#include "architecture_measurement/impl/space_analytic_geometry_ex.hpp"
#include "architecture_measurement/impl/cloud_pretreatment_ex.hpp"
#include "architecture_measurement/impl/cloud_header_ex.hpp"

#include "architecture_measurement/masonry.hpp"
#include "architecture_measurement/surface_flatness.hpp"

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "opencv2/core.hpp"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/PointIndices.h"
#include "pcl/ModelCoefficients.h"

/**
 * @brief 
 * 
 */
namespace am
{

/**
 * @brief Mortar joint identification of masonry wall
 * 
 */
class MasonrySurfaceFlatness
{
    public:

        MasonrySurfaceFlatness();

        /**
         * @brief Destroy the Masonry Surface Flatness object
         * 
         */
        virtual ~MasonrySurfaceFlatness();

        /**
         * @brief screen mortar joint from point cloud
         * 
         * @param PointCloudPtr origin point cloud
         * @param PointCloudFilterPtr filtered point cloud
         * @param indicesFilterPtr filtered point cloud indices
         * @param mortarJointImage mrotar joint image
         * @param engineering
         * @param object measured object
         * @param method matching rules between image pixel and point cloud index 
         * @return int 
         */
        virtual std::vector<float> Calculate(
            pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudPtr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudFilterPtr,
            pcl::PointIndices::Ptr indicesFilterPtr, 
            cv::Mat& mortarJointImage,
            const int& engineering,
            const int& object, 
            const int& method);

        virtual int Pretreatment(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr,
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& pointCloudPtrVec,
            std::vector<pcl::PointIndices::Ptr>& pointIndicesPtrVec,
            const Eigen::Matrix3f& W_R_C,
            const int& engineering,
            const int& object);

        virtual int SortCloud(
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& pointCloudPtrVec, 
            int& index,
            const int& engineering,
            const int& object);

        virtual std::vector<float> VirtualRuler(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr, 
            const Eigen::Matrix3f& W_R_C);

        virtual float Statistic(pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr, pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudInliersPtr);
        
        int SetI_R_C(const Eigen::Matrix3f& I_R_C);

        int SetAngleXY(const float& angleX, const float& angleY);

        int SetFileName(const std::string& fileName);

        int SetImageWidth(const float& imageWidth);

        int SetImageHeight(const float& imageHeight);

    private:
        Masonry _masonry; /**< */
        Inclinometer _inclinometer; /**< */
        PointCloudGUI _pGUI; /**< */  

        float _imageWidth; /**< */
        float _imageHeight; /**< */
        Eigen::Matrix3f _I_R_C; /**< */
};

} //namespace am

#endif //_MASONRY_SURFACE_FLATNESS_H