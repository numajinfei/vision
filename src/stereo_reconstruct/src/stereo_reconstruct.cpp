#include "stereo_reconstruct/stereo_reconstruct.hpp"

namespace stereo_reconstruct
{

class StereoReconstruct::_Impl
{
public:
    explicit _Impl(StereoReconstruct* ptr) : _node(ptr)
    {
    }

    ~_Impl()
    {
    }

private:
    StereoReconstruct* _node;
};

StereoReconstruct::StereoReconstruct(const rclcpp::NodeOptions& options) : Node("stereo_reconstruct_node", options)
{
    _init = std::thread(&StereoReconstruct::_Init, this);
}

StereoReconstruct::~StereoReconstruct()
{
    _init.join();

    _srv.reset();
    _sub.reset();
    _impl.reset();
    _pub.reset();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "stereo_reconstruct destroyed successfully");
}

void StereoReconstruct::_Init() try
{
    _InitializeParameters();

    _UpdateParameters();

    _pub = this->create_publisher<std_msgs::msg::String>(_pubName, 10);

    _impl = std::make_unique<_Impl>(this);

    _sub = this->create_subscription<std_msgs::msg::String>(_subName, 10, std::bind(&StereoReconstruct::_Sub, this, std::placeholders::_1));

    _srv = this->create_service<std_srvs::srv::Trigger>(_srvName, std::bind(&StereoReconstruct::_Srv, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "stereo_reconstruct initialized successfully");
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in stereo_reconstruct initializer: %s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in stereo_reconstruct initializer: unknown");
    rclcpp::shutdown();
}

void StereoReconstruct::_Sub(std_msgs::msg::String::UniquePtr /*ptr*/) try
{
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in stereo_reconstruct subscription: %s", e.what());
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in stereo_reconstruct subscription: unknown");
}

void StereoReconstruct::_Srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> /*response*/) try
{
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in stereo_reconstruct service: %s", e.what());
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in stereo_reconstruct service: unknown");
}

void StereoReconstruct::_InitializeParameters()
{
    //this->declare_parameter("");
}

void StereoReconstruct::_UpdateParameters()
{
    //this->get_parameter("", );
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(stereo_reconstruct::StereoReconstruct)

