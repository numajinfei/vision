
#pragma once
#ifndef _FUNDAMENTAL_H
#define _FUNDAMENTAL_H

#include <string>
#include <cmath>
#include <map>
#include <utility>
#include <unordered_map>

#include "Eigen/Core"

#include "opencv2/core.hpp"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/PointIndices.h"
#include "pcl/ModelCoefficients.h"

namespace am
{

/**
 * @brief degree convert to radian
 * 
 * @param degree angle degree
 * @return Type angle radian
 */
double Radian(const double& degree);

/**
 * @brief 
 * 
 * @param degree 
 * @return float 
 */
float Radian(const float& degree);

/**
 * @brief radian convert to degree
 * 
 * @param radian angle radian
 * @return Type angle degree
 */
double Degree(const double& radian);

/**
 * @brief 
 * 
 * @param radian 
 * @return float 
 */
float Degree(const float& radian);

/**
 * @brief 
 * 
 * @param value 
 * @param ending 
 * @return true 
 * @return false 
 */
bool EndsWith(const std::string& value, const std::string& ending);

/**
 * @brief engineer type
 * 
 */
enum Engineer
{
    NORMAL = 0, /**< general engineering */
    CONCERTE = 1, /**< concerte engineering */
    MASONRY = 2, /**< masonry engineering */
    PLASTER = 3, /**< plaster engineering */
    GROUND_BASE = 4, /**< ground base engineering */
    GROUND_SURFACE = 5, /**< ground surface engineering */
    PARTITION = 6, /**< partition engineering */
    VENEER = 7 /**< veneer engineering */
};

enum Object
{
    SIDEWALL = 0, 
    GROUND = 1, 
    ROOF = 2,
    PILLAR = 3,
    INTERNAL_ANGLE = 4,
    EXTERNAL_ANGLE = 5,
    DOOR = 6, 
    WINDOW = 7
};

std::unordered_map<std::string, int> LUTEngineering();
std::unordered_map<std::string, int>  LUTOption();

constexpr float THRESHOLD = std::sqrt(2)/2.0;

constexpr float THRESHOLD_DISTANCE_FROM_POINT_TO_LINE = 50;

class CombinationDate
{
    public:
        CombinationDate();

        ~CombinationDate();

        int Complete();

    public:   
        pcl::PointCloud<pcl::PointXYZI>::Ptr _pointCloudPtr;
        cv::Mat _image;
        long _pointCloudID;
        long _imageID;
};

// class VerticalBoundary
// {
//     public:
//         VerticalBoundary(
//             pcl::ModelCoefficients& coeffs, 
//             pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr);

//         ~VerticalBoundary();
    
//     public:            
//         pcl::ModelCoefficients _coeffs;
//         pcl::PointCloud<pcl::PointXYZI>::Ptr _pointCloudPtr;
//         float _x;
//         float _y;
//         float _z;
//         float _length;
//         float _limitZS;
//         float _limitZL;

//     private:
//         void CalculateLineLength(); 
//         void ClaculateLineCenter();   
//         void CalculateLimitsZ();    
// };

// bool SortVerticalBoundaryXL(VerticalBoundary& a, VerticalBoundary& b);

// bool SortVerticalBoundaryXG(VerticalBoundary& a, VerticalBoundary& b);

// bool SortVerticalBoundaryYL(VerticalBoundary& a, VerticalBoundary& b);

// bool SortVerticalBoundaryYG(VerticalBoundary& a, VerticalBoundary& b);

// bool SortVerticalBoundaryZL(VerticalBoundary& a, VerticalBoundary& b);

// bool SortVerticalBoundaryZG(VerticalBoundary& a, VerticalBoundary& b);

// bool SortLengthL(VerticalBoundary& a, VerticalBoundary& b);

// bool SortLengthG(VerticalBoundary& a, VerticalBoundary& b);


// struct IntersectionPlaneIndices
// {
//     int index1;
//     int index2;
// };
// typedef IntersectionPlaneIndices InternalAngleIndices;
// typedef IntersectionPlaneIndices ExternalAngleIndices;

// struct IntersectionLine
// {
//     IntersectionPlaneIndices indices;
//     Eigen::Vector3f point;
//     Eigen::Vector3f vector;
//     float limitS;
//     float limitL;
//     float limitM;
// };

// bool SortIntersectionLineXC(IntersectionLine& line1, IntersectionLine& line2);

// bool SortIntersectionLineYC(IntersectionLine& line1, IntersectionLine& line2);

// bool SortIntersectionLineZC(IntersectionLine& line1, IntersectionLine& line2);

struct OpeningData
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtr;
    long id;
    std::string engineering;
    std::string option;
    float angle;
    float angleX;
    float angleY;
    int frameID;
};

// bool SortPointCloudXL(pcl::PointXYZI& point1, pcl::PointXYZI& point2);

// bool SortPointCloudXG(pcl::PointXYZI& point1, pcl::PointXYZI& point2);

// bool SortPointCloudYL(pcl::PointXYZI& point1, pcl::PointXYZI& point2);

// bool SortPointCloudYG(pcl::PointXYZI& point1, pcl::PointXYZI& point2);

// bool SortPointCloudZL(pcl::PointXYZI& point1, pcl::PointXYZI& point2);

// bool SortPointCloudZG(pcl::PointXYZI& point1, pcl::PointXYZI& point2);

} //namespace am

#endif //_FUNDAMENTAL_H