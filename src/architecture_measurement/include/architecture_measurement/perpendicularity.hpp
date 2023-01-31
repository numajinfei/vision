#pragma once
#ifndef _PERPENDICULARITY_H
#define _PERPENDICULARITY_H

#include "architecture_measurement/impl/fundamental.hpp"

#include <string>
#include <vector>

#include "Eigen/Core"

#include "pcl/point_types.h"
#include "pcl/common/distances.h"
#include "pcl/ModelCoefficients.h"

// #include "yaml-cpp/yaml.h

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "shared_interfaces/msg/float64_array.hpp"
#include "shared_interfaces/msg/roll_pitch_yaw.hpp"
#include "shared_interfaces/msg/point_cloud_c.hpp"

namespace am
{

class VerticalBoundary
{
    public:
        VerticalBoundary();

        VerticalBoundary(
            pcl::ModelCoefficients& coeffs, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr);

        ~VerticalBoundary();
    
    public:            
        pcl::ModelCoefficients _coeffs;
        pcl::PointCloud<pcl::PointXYZI>::Ptr _pointCloudPtr;
        float _x;
        float _y;
        float _z;
        float _length;
        float _limitZS;
        float _limitZL;

    private:
        void CalculateLineLength(); 
        void ClaculateLineCenter();   
        void CalculateLimitsZ();    
};

bool SortVerticalBoundaryXL(VerticalBoundary& a, VerticalBoundary& b);

bool SortVerticalBoundaryXG(VerticalBoundary& a, VerticalBoundary& b);

bool SortVerticalBoundaryYL(VerticalBoundary& a, VerticalBoundary& b);

bool SortVerticalBoundaryYG(VerticalBoundary& a, VerticalBoundary& b);

bool SortVerticalBoundaryZL(VerticalBoundary& a, VerticalBoundary& b);

bool SortVerticalBoundaryZG(VerticalBoundary& a, VerticalBoundary& b);

bool SortLengthL(VerticalBoundary& a, VerticalBoundary& b);

bool SortLengthG(VerticalBoundary& a, VerticalBoundary& b);

bool SortPointCloudXL(pcl::PointXYZI& point1, pcl::PointXYZI& point2);

bool SortPointCloudXG(pcl::PointXYZI& point1, pcl::PointXYZI& point2);

bool SortPointCloudYL(pcl::PointXYZI& point1, pcl::PointXYZI& point2);

bool SortPointCloudYG(pcl::PointXYZI& point1, pcl::PointXYZI& point2);

bool SortPointCloudZL(pcl::PointXYZI& point1, pcl::PointXYZI& point2);

bool SortPointCloudZG(pcl::PointXYZI& point1, pcl::PointXYZI& point2);

class Perpendicularity : public rclcpp::Node
{
    public:
        explicit Perpendicularity(const rclcpp::NodeOptions& option);
        ~Perpendicularity();

    private:
        void _Init();
        void _InitializeParameters();
        void _UpdateParameters();
        void _SubPointCloud(shared_interfaces::msg::PointCloudC::UniquePtr ptr);
        void _SubRpy(shared_interfaces::msg::RollPitchYaw::UniquePtr ptr);
        void _PubResult(const std::vector<double>& result);
        void _PubStatus(const std::string& status);

    private:
        double _roll, _pitch, _yaw;

        std::vector<std::string> _engineeringVec;
        std::vector<std::string> _optionVec;

        int _status;

        class _Impl;
        std::unique_ptr<_Impl> _impl;

        const char* _subPointCloudName = "~/pointcloud";
        rclcpp::Subscription<shared_interfaces::msg::PointCloudC>::SharedPtr _subPointCloud;

        const char* _subRpyName = "~/rollPitchYaw";
        rclcpp::Subscription<shared_interfaces::msg::RollPitchYaw>::SharedPtr _subRpy;

        const char* _pubResultName = "~/result";
        rclcpp::Publisher<shared_interfaces::msg::Float64Array>::SharedPtr _pubResult;
        
        const char* _pubStatusName = "~/status";
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pubStatus;

        std::thread _initThread;
};

}//namespace am

#endif //_PERPENDICULARITY_H