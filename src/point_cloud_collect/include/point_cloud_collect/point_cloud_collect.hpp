#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace point_cloud_collect
{

class PointCloudCollect : public rclcpp::Node
{
public:
    explicit PointCloudCollect(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PointCloudCollect();

private:
    void _Sub(sensor_msgs::msg::PointCloud2::UniquePtr ptr);

private:
    const char* _pubName = "~/points";
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;

    class _Impl;
    std::unique_ptr<_Impl> _impl;

    const char* _subName = "~/line";
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub;
};

}

