#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "shared_interfaces/msg/line_center.hpp"

namespace laser_line_reconstruct
{

class LaserLineReconstruct : public rclcpp::Node
{
public:
    explicit LaserLineReconstruct(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LaserLineReconstruct();

    void Publish(sensor_msgs::msg::PointCloud2::UniquePtr& msg)
    {
        _pub->publish(std::move(msg));
    }

private:
    const char* _pubName = "~/line";
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;

    class _Impl;
    std::unique_ptr<_Impl> _impl;

    const char* _subLName = "~/line_l";
    const char* _subRName = "~/line_r";
    rclcpp::CallbackGroup::SharedPtr _cbgL;
    rclcpp::CallbackGroup::SharedPtr _cbgR;
    rclcpp::Subscription<shared_interfaces::msg::LineCenter>::SharedPtr _subL;
    rclcpp::Subscription<shared_interfaces::msg::LineCenter>::SharedPtr _subR;
};

}
