#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace image_file_reader
{

class ImageFileReader : public rclcpp::Node
{
public:
    explicit ImageFileReader(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ImageFileReader();

private:
    void _InitializeParameters();
    void _UpdateParameters();

private:
    int _frameId = 0;
    std::string _imageFile;

    const char* _pubName = "~/image";
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub;

    const char* _srvStartName = "~/start";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvStart;
};

}
