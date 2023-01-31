#pragma once
#ifndef _FLOOR_HEIGHT_H
#define _FLOOR_HEIGHT_H

#include <string>
#include <vector>
#include <memory>
#include <thread>

#include "Eigen/Core"

#include "rclcpp/rclcpp.hpp"
// #include "std_srvs/srv/trigger.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"
#include "shared_interfaces/msg/float64_array.hpp"
#include "shared_interfaces/msg/laser_ranger.hpp"
#include "shared_interfaces/msg/roll_pitch_yaw.hpp"
#include "std_msgs/msg/string.hpp"
// #include "shared_interfaces/srv/trigger_op.hpp"


namespace am
{

class FloorHeight : public rclcpp::Node
{
    public:
        explicit FloorHeight(const rclcpp::NodeOptions& option = rclcpp::NodeOptions());
        virtual ~FloorHeight();

    private:
        void _Init();
        void _InitializeParameters();
        void _UpdateParameters();

        void _SubRpy(shared_interfaces::msg::RollPitchYaw::UniquePtr ptr);
        void _SubLaserRanger(shared_interfaces::msg::LaserRanger::UniquePtr ptr);
        void _PubResult(const std::vector<double>& result);
        void _PubStatus(const std::string& status);

    private:
        
        double _laserRanger;
        double _roll, _pitch, _yaw;
        int _status;

        class _Impl;
        std::unique_ptr<_Impl> _impl;

        const char* _subRpyName = "~/rollPitchYaw";
        rclcpp::Subscription<shared_interfaces::msg::RollPitchYaw>::SharedPtr _subRpy;

        const char* _subLaserRangerName = "~/laserRanger";
        rclcpp::Subscription<shared_interfaces::msg::LaserRanger>::SharedPtr _subLaserRanger;

        const char* _pubResultName = "~/result";
        rclcpp::Publisher<shared_interfaces::msg::Float64Array>::SharedPtr _pubResult;

        const char* _pubStatusName = "~/status";
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pubStatus;

        std::thread _initThread;
};

}//namespace am

#endif //_FLOOR_HEIGHT_H