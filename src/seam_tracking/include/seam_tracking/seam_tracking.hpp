#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "shared_interfaces/msg/modbus_coord.hpp"

namespace seam_tracking
{

class SeamTracking : public rclcpp::Node
{
public:
    explicit SeamTracking(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SeamTracking();

    void PublishCoord(shared_interfaces::msg::ModbusCoord::UniquePtr& ptr)
    {
        _pubCoord->publish(std::move(ptr));
    }

    void PublishLine(sensor_msgs::msg::PointCloud2::UniquePtr& ptr)
    {
        _pubLine->publish(std::move(ptr));
    }

    void PublishMarker(visualization_msgs::msg::Marker::UniquePtr& ptr)
    {
        _pubMarker->publish(std::move(ptr));
    }

private:
    void _Init();
    void _InitializeParameters();
    void _UpdateParameters();
    void _Sub(sensor_msgs::msg::Image::UniquePtr ptr);

private:
    const char* _pubNameLine = "~/line";
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pubLine;

    const char* _pubNameCoord = "~/coord";
    rclcpp::Publisher<shared_interfaces::msg::ModbusCoord>::SharedPtr _pubCoord;

    const char* _pubNameMarker = "~/marker";
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pubMarker;

    class _Impl;
    std::unique_ptr<_Impl> _impl;

    const char* _subName = "~/image";
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub;

    std::thread _init;
};

}

