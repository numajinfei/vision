#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace image_file_writer
{

class ImageFileWriter : public rclcpp::Node
{
public:
    explicit ImageFileWriter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ImageFileWriter();

private:
    void _InitializeParameters();
    void _UpdateParameters();
    void _Sub(sensor_msgs::msg::Image::UniquePtr ptr);

private:
    int _count = 0;
    const char* _subName = "~/image";
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub;
};

}

