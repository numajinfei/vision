#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace hz_sensor
{

class HzSensor : public rclcpp::Node
{
public:
    explicit HzSensor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~HzSensor();

private:
    void _InitializeParameters();
    void _UpdateParameters();

private:
    int _imgCount = 0;
    int _pclCount = 0;
    std::string _imgFrame;
    std::string _pclFrame;

    const char* _subImgName = "~/image";
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subImg;

    const char* _subPclName = "~/points";
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subPcl;
};

}
