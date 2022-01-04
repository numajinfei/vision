#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace camera_galaxy
{

class CameraGalaxy : public rclcpp::Node
{
public:
    explicit CameraGalaxy(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CameraGalaxy();

    void PublishL(sensor_msgs::msg::Image::UniquePtr& ptr)
    {
        _pubL->publish(std::move(ptr));
    }

    void PublishR(sensor_msgs::msg::Image::UniquePtr& ptr)
    {
        _pubR->publish(std::move(ptr));
    }

private:
    void _Init();
    void _Start(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void _Stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

private:
    const char* _pubLName = "~/image_l";
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pubL;

    const char* _pubRName = "~/image_r";
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pubR;

    class _Impl;
    std::unique_ptr<_Impl> _impl;

    const char* _srvStartName = "~/start";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvStart;

    const char* _srvStopName = "~/stop";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvStop;

    std::thread _init;
};

}
