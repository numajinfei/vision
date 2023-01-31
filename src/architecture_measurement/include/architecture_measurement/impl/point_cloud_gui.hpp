#pragma once
#ifndef _POINT_CLOUD_GUI_H
#define _POINT_CLOUD_GUI_H

#include "architecture_measurement/impl/file_io.hpp"
#include "architecture_measurement/impl/cloud_pretreatment.hpp"

#include <iostream>
#include <string>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/PointIndices.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/filters/voxel_grid.h"

/**
 * @brief 
 * 
 */
namespace am
{

/**
 * @brief 
 * 
 */
class PointCloudGUI
{
    public:
        /**
         * @brief Construct a new Point Cloud G U I object
         * 
         */
        PointCloudGUI();

        /**
         * @brief Destroy the Point Cloud G U I object
         * 
         */
        ~PointCloudGUI();

        /**
         * @brief 
         * 
         * @return int 
         */
        int SavePointCloud();

        /**
         * @brief Set the Point Cloud Ground object
         * 
         * @param pointCloudPtr 
         * @return int 
         */
        int SetPointCloudGround(pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr);

        /**
         * @brief Set the Point Cloud Rulers object
         * 
         * @param pointCloudPtr 
         * @return int 
         */
        int SetPointCloudRulers(pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr);

        /**
         * @brief Set the File Name object
         * 
         * @param fileName 
         * @return int 
         */
        int SetFileName(const std::string& fileName);
    
    private:
        /**
         * @brief 
         * 
         * @param pointCloudPtr 
         * @return int 
         */
        int DuplicateRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr);

        /**
         * @brief 
         * 
         * @return int 
         */
        int Transformation(pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr);

    private:
        pcl::PointCloud<pcl::PointXYZI>::Ptr _pointCloudPtr; /**< */
        std::string _fileName; /**< */
};

}//namespace am

#endif //_POINT_CLOUD_GUI_H