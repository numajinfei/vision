#include "hz_sensor/hz_sensor.hpp"

namespace hz_sensor
{

HzSensor::HzSensor(const rclcpp::NodeOptions& options) : Node("hz_sensor_node", options)
{
    _InitializeParameters();

    _UpdateParameters();

    _subImg = this->create_subscription<sensor_msgs::msg::Image>(_subImgName, 10, [this](sensor_msgs::msg::Image::UniquePtr msg)
    {
        _imgCount++;
        _imgFrame = msg->header.frame_id;
    });

    _subPcl = this->create_subscription<sensor_msgs::msg::PointCloud2>(_subPclName, 10, [this](sensor_msgs::msg::PointCloud2::UniquePtr msg)
    {
        _pclCount++;
        _pclFrame = msg->header.frame_id;
    });
}

HzSensor::~HzSensor()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Image topic, count:[%d] frame:[%s]", _imgCount, _imgFrame.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Point topic, count:[%d] frame:[%s]", _pclCount, _pclFrame.c_str());
}

void HzSensor::_InitializeParameters()
{
    //this->declare_parameter("image_topic", "image");
    //this->declare_parameter("point_topic", "point");
}

void HzSensor::_UpdateParameters()
{
    //this->get_parameter("image_topic", _imgTopic);
    //this->get_parameter("point_topic", _pclTopic);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hz_sensor::HzSensor)