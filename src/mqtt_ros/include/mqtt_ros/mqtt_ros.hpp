#ifndef _MQTT_ROS_H
#define _MQTT_ROS_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include<thread>

namespace mqtt_ros
{

class MqttRos : public rclcpp::Node
{
    public:
        explicit MqttRos(const rclcpp::NodeOptions& options = rclcpp::NodeOptions() );
        ~MqttRos();

    private:
        void _Init();
        void _InitializeParameters();
        void _UpdateParameters();
        void _PubMeasurementRequest(std_msgs::msg::String::UniquePtr& ptr);
        void _SubResult(std_msgs::msg::String::UniquePtr ptr); 
        void _SubStatus(std_msgs::msg::String::UniquePtr ptr);

    private:
        int _status;

        class _Impl;
        std::unique_ptr<_Impl> _impl;

        const char* _pubMeasurementRequestName = "~/measurement_request";
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pubMeasurementRequest;

        const char* _subResultName = "~/result";
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subResult;

        const char* _subStatusName = "~/status";
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subStatus;

        std::thread _initThread;
};

}//namespace mqtt_ros

#endif //_MQTT_ROS_H

