#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace mqtt_ros
{

class MqttRos : public rclcpp::Node
{
public:
    explicit MqttRos(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MqttRos();
    void Publish(std_msgs::msg::String::UniquePtr& ptr);

private:
    void _Init();
    void _Sub(std_msgs::msg::String::UniquePtr ptr);

private:
    const char* _pubName = "~/request";
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;

    class _Impl;
    std::unique_ptr<_Impl> _impl;

    const char* _subName = "~/response";
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;

    std::thread _init;
};

}

