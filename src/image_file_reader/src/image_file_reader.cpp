#include "image_file_reader/image_file_reader.hpp"
#include "opencv2/opencv.hpp"

namespace image_file_reader
{

ImageFileReader::ImageFileReader(const rclcpp::NodeOptions& options) : Node("image_file_reader_node", options)
{
    _InitializeParameters();

    _UpdateParameters();

    _pub = this->create_publisher<sensor_msgs::msg::Image>(_pubName, 10);

    _srvStart = this->create_service<std_srvs::srv::Trigger>(_srvStartName, [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try
        {
            _UpdateParameters();
            auto img = cv::imread(_imageFile, cv::IMREAD_GRAYSCALE);
            if(img.data == NULL)
            {
                response->success = false;
                response->message = "Fail: read image";
                return;
            }
            sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
            msg->header.stamp = this->now();
            msg->header.frame_id = std::to_string(_frameId++);
            msg->height = img.rows;
            msg->width = img.cols;
            msg->encoding = "mono8";
            msg->is_bigendian = false;
            msg->step = img.cols;
            msg->data.resize(img.rows * img.cols);
            memcpy(msg->data.data(), img.data, img.rows * img.cols);
            _pub->publish(std::move(msg));

            response->success = true;
            response->message = "Success: read image";
        }
        catch(...)
        {
            response->success = false;
            response->message = "Fail: read image";
        }
    });
}

ImageFileReader::~ImageFileReader()
{
}

void ImageFileReader::_InitializeParameters()
{
    this->declare_parameter("image_file");
}

void ImageFileReader::_UpdateParameters()
{
    this->get_parameter("image_file", _imageFile);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(image_file_reader::ImageFileReader)
