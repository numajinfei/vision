#ifndef _CONDITION_MONITORING_H
#define _CONDITION_MONITORING_H

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "shared_interfaces/msg/float64_array.hpp"
#include "shared_interfaces/srv/trigger_op.hpp"

namespace condition_monitoring
{

enum STATUS {STATUS_READY, STATUS_BUSY, STATUS_ERROR};

class ConditionMonitoring : public rclcpp::Node
{
    public:
        explicit ConditionMonitoring(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~ConditionMonitoring();

    private:    
        void _Init();
        void _InitializeParameters();
        void _UpdateParameters();
        void _SubStatus(std_msgs::msg::String::UniquePtr ptr);
        void _PubStatus(const std::string& status);

    private:
        int _status;
        
        class _Impl;
        std::unique_ptr<_Impl> _impl;

        const char* _pubStatusName = "~/status_publisher";
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pubStatus;

        const char* _subStatusName = "~/status_subscription";
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subStatus;

        std::thread _initThread;
};

}//namespace condition_monitoring

#endif //_CONDITION_MONITORING_H

