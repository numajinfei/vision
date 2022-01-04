#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace camera_spinnaker
{

class CameraSpinnaker : public rclcpp::Node
{
public:
    explicit CameraSpinnaker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CameraSpinnaker();
    
    void Publish(sensor_msgs::msg::Image::UniquePtr& ptr)
    {
        _pub->publish(std::move(ptr));
    }

private:
    void _Init();
    void _InitializeParameters();
    void _UpdateParameters();
    void _SrvStart(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void _SrvStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

private:
    const char* _pubName = "~/image";
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub;

    class _Impl;
    std::unique_ptr<_Impl> _impl;

    const char* _srvStartName = "~/start";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvStart;
    
    const char* _srvStopName = "~/stop";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvStop;

    std::thread _init;
};

}

