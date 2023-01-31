#pragma once
#ifndef _CLOUD_HEADER_H
#define _CLOUD_HEADER_H

#include <memory>

#include "Eigen/Core"

#include "yaml-cpp/yaml.h"

#include "pcl/point_types.h"
#include "pcl/common/distances.h"

namespace am
{

enum SortMethod{SIZE = 0, NORMAL_VECTOR = 1, MASS_CENTER = 2};

class CloudHeader
{
    private:
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        int index;
        std::size_t size;
        Eigen::Vector3f normalVector;
        Eigen::Vector4f planeCoeff;
        Eigen::Vector3f massCenter;

        class ParamsConfig; 
    private:
        void Plane();
        void CalculateOBB();

    public:
        CloudHeader();
        CloudHeader(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _cloud, const int& _index);
        CloudHeader(const CloudHeader& cloud_header);
        CloudHeader& operator=(const CloudHeader& cloud_header);
        ~CloudHeader();

        // void SetProperty(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _cloud, const int& _index);

        inline pcl::PointCloud<pcl::PointXYZI>::Ptr GetPointCloudPtr()
        {
            return cloud;
        }

        inline int GetIndex()
        {
            return index;
        }

        inline long int GetSize()
        {
            return size;
        }

        inline Eigen::Vector3f GetNormalVector()
        {
            return normalVector;
        }

        inline Eigen::Vector4f GetPlaneCoeff()
        {
            return planeCoeff;
        }

        inline Eigen::Vector3f GetMassCenter()
        {
            return massCenter;
        }
};

class CloudHeader::ParamsConfig
{
    private:
        float distanceThreshold;

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

        inline float GetDistanceThreshold()
        {
            return distanceThreshold;
        }
};

using CloudHeaderVector = std::vector<CloudHeader>;

bool SortSize(CloudHeader& ch1, CloudHeader& ch2);

bool SortNormalVectorL(CloudHeader& ch1, CloudHeader& ch2);

bool SortNormalVectorG(CloudHeader& ch1, CloudHeader& ch2);

bool SortMassCenterXL(CloudHeader& ch1, CloudHeader& ch2);

bool SortMassCenterXG(CloudHeader& ch1, CloudHeader& ch2);

bool SortMassCenterYL(CloudHeader& ch1, CloudHeader& ch2);

bool SortMassCenterYG(CloudHeader& ch1, CloudHeader& ch2);

bool SortMassCenterZL(CloudHeader& ch1, CloudHeader& ch2);

bool SortMassCenterZG(CloudHeader& ch1, CloudHeader& ch2);

}//namespace am

#endif //_CLOUD_HEADER_H