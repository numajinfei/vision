#pragma once
#ifndef _MASONRY_PERPENDICULARITY_H
#define _MASONRY_PERPENDICULARITY_H

#include "architecture_measurement/impl/inclinometer.hpp"
#include "architecture_measurement/impl/point_cloud_gui.hpp"
#include "architecture_measurement/impl/fundamental.hpp"
#include "architecture_measurement/impl/space_analytic_geometry_ex.hpp"
#include "architecture_measurement/impl/cloud_pretreatment_ex.hpp"
#include "architecture_measurement/impl/cloud_header_ex.hpp"

#include "architecture_measurement/masonry.hpp"
#include "architecture_measurement/perpendicularity.hpp"

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "opencv2/core.hpp"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/PointIndices.h"
#include "pcl/ModelCoefficients.h"


namespace am
{

class MasonryPerpendicularity
{
    public:

        MasonryPerpendicularity();

        virtual ~MasonryPerpendicularity();

        std::vector<float> Calculate(
            pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudPtr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudFilterPtr,
            pcl::PointIndices::Ptr indicesFilterPtr, 
            cv::Mat& mortarJointImage,
            const int& engineering,
            const int& object, 
            const int& method);

        int Pretreatment(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr,
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& pointCloudPtrVec,
            std::vector<pcl::PointIndices::Ptr>& pointIndicesPtrVec,
            const Eigen::Matrix3f& W_R_C,
            const int& engineering,
            const int& object);
        
        int SetImageWidth(const float& imageWidth);

        int SetImageHeight(const float& imageHeight);

        int SortCloud(
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& pointCloudPtrVec,
            int& index,
            const int& engineering, 
            const int& object);

        std::vector<float> VirtualRulerSideWall(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtrOptimal,
            pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtrRulers,
            const Eigen::Matrix3f& W_R_C,
            const int& engineering,
            const int& object);

        std::vector<float> VirtualRulerPillar(
            pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtrOptimal,
            pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtrRulers,
            const Eigen::Matrix3f& W_R_C,
            const int& engineering,
            const int& object);

        float Statistic(pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr);

        int SetI_R_C(const Eigen::Matrix3f& I_R_C);

        int SetAngleXY(const float& angleX, const float& angleY);

        int SetFileName(const std::string& fileName);

        int SetHyperParameter(const float& hyperparameter);

    public:
        Masonry _masonry; /**< */
        Inclinometer _inclinometer; /**< */
        PointCloudGUI _pGUI; /**< */

        Eigen::Matrix3f _I_R_C; /**< */
        float _hyperparameter; /**< */  
        float _imageWidth; /**< */
        float _imageHeight; /**< */
};

}//namespace am

#endif //_MASONRY_PERPENDICULARITY_H