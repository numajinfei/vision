#pragma once
#ifndef _PILLAR_SECTION_SIZE_H
#define _PILLAR_SECTION_SIZE_H

#include <string>
#include <thread>

#include "Eigen/Core"

// #include "yaml-cpp/yaml.h"

#include "pcl/point_types.h"
#include "pcl/common/distances.h"
#include "pcl/ModelCoefficients.h"

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
        VerticalBoundary(pcl::ModelCoefficients _coef, pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud);
        ~VerticalBoundary();

        inline float GetX()
        {
            return x;
        }

        inline float GetY()
        {
            return y;
        }

        inline float GetZ()
        {
            return z;
        }

        inline float GetLength()
        {
            return length;
        }
    
    private:
        void CalculateLineLength(); 
        void ClaculateLineCenter();   
        void CalculateLimitsZ();    

    public:            
        pcl::ModelCoefficients coef;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        float x;
        float y;
        float z;
        float length;
        float limit_z_s;
        float limit_z_l;
};

class PillarSectionSize : public rclcpp::Node
{
    public:
        explicit PillarSectionSize(const rclcpp::NodeOptions& option);
        ~PillarSectionSize();

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

#endif //_PILLAR_SECTION_SIZE_H