#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace stereo_reconstruct
{

class StereoReconstruct : public rclcpp::Node
{
public:
    explicit StereoReconstruct(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~StereoReconstruct();

private:
    void _Init();
    void _InitializeParameters();
    void _UpdateParameters();
    void _Sub(std_msgs::msg::String::UniquePtr ptr);//TODO
    void _Srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);//TODO

private:
    const char* _pubName = "~/pub";//TODO
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;

    class _Impl;
    std::unique_ptr<_Impl> _impl;

    const char* _subName = "~/sub";//TODO
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;

    const char* _srvName = "~/srv";//TODO
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srv;

    std::thread _init;
};

}

