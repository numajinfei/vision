#pragma once
#ifndef _MASONRY_H
#define _MASONRY_H

#include "architecture_measurement/impl/cloud_pretreatment.hpp"
#include "architecture_measurement/impl/fundamental.hpp"

#include "opencv2/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"

#define AM_TEST

/**
 * @brief namespace architecture_measurement
 * 
 */
namespace am
{

/**
 * @brief detecting method
 * 
 */
enum Method
{
    POINTCLOUD = 0, /**< 3d method */
    IMAGE = 1, /**< 2d method */
    POINTCLOUD_AND_IMAGE = 2 /**< combination of 3d and 2d method */
};

/**
 * @brief matching rules between image pixel and point cloud index 
 * 
 */
enum Correlation
{
    COL_MAJOR = 0, /**< index = x * height + y */
    ROW_MAJOR = 1 /**< index = y * width + x */
};

/**
 * @brief 
 * 
 */
class Masonry
{
    public:
        /**
         * @brief Construct a new Masonry Wall object
         * 
         */
        Masonry();

        /**
         * @brief Destroy the Masonry Wall object
         * 
         */
        ~Masonry();

        /**
         * @brief calculate point cloud index from pixel coordinate
         * 
         * @param point pixel coordinate
         * @param index point cloud index
         * @param method matching rules between image pixel and point cloud index 
         * @return int 
         */
        int Search(const cv::Point& point, int& index, const int& method);

        /**
         * @brief calculate pixel coordinate from point cloud index
         * 
         * @param index point cloud index
         * @param point pixel coordinate
         * @param method matching rules between image pixel and point cloud index 
         * @return int 
         */
        int Search(const int& index, cv::Point& point, const int method);

        /**
         * @brief Create a Mask Map object
         * 
         * @param indices optimal point cloud indices
         * @param maskMap mask image
         * @return int 
         */
        int CreateMaskMap(pcl::PointIndices::Ptr& indices, cv::Mat& maskMap);

        /**
         * @brief extract mortar joint point
         * 
         * @param mortarJointImage mortar joint image
         * @param maskImage mask image
         * @param indicesPtr mortar joint point cloud indices
         * @return int 
         */
        int ExtractMortarJoint(cv::Mat& mortarJointImage, cv::Mat& maskImage, pcl::PointIndices::Ptr& indicesPtr);

        /**
         * @brief least squares plane fitting
         * 
         * @param pointCloudPtr point cloud shared ptr
         * @return pcl::ModelCoefficients 
         */
        pcl::ModelCoefficients LeastSquaresPlaneFitting(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtr);

        /**
         * @brief least squares quadric plane fitting
         * 
         * @param pointCloudPtr point cloud shared ptr
         * @return pcl::ModelCoefficients 
         */
        pcl::ModelCoefficients LeastSquaresQuadricPlaneFitting(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtr);

        /**
         * @brief image median filter
         * 
         * @param src src image
         * @param dst dst image
         * @return int 
         */
        int MedianFilter(cv::Mat& src, cv::Mat& dst);

        /**
         * @brief image dilate filter
         * 
         * @param src src image
         * @param dst dst image
         * @return int 
         */
        int DilateFilter(cv::Mat& src, cv::Mat& dst);

        int SetImageWidth(const int& imageWidth);

        int SetImageHeight(const int& imageHeight);

    private:
        int _imageHeight; /**< image rows */
        int _imageWidth; /**< image cols */
};

} //namespace am

#endif //_MASONRY_H
