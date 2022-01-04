#include "image_file_writer/image_file_writer.hpp"
#include "opencv2/opencv.hpp"

namespace image_file_writer
{

ImageFileWriter::ImageFileWriter(const rclcpp::NodeOptions& options) : Node("image_file_writer_node", options)
{
    _InitializeParameters();

    _UpdateParameters();

    _sub = this->create_subscription<sensor_msgs::msg::Image>(_subName, 1, std::bind(&ImageFileWriter::_Sub, this, std::placeholders::_1));
}

ImageFileWriter::~ImageFileWriter()
{
}

void ImageFileWriter::_Sub(sensor_msgs::msg::Image::UniquePtr ptr) try
{
    if(ptr->header.frame_id == "-1")
        return;

    if(ptr->encoding != "mono8")
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Can not handle color image");
        return;
    }

    cv::Mat img(ptr->height, ptr->width, CV_8UC1, ptr->data.data());
    std::string name = this->get_name() + std::to_string(_count++) + ".png";
    cv::imwrite(name, img);
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in image_file_writer subscription: %s", e.what());
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in image_file_writer subscription: unknown");
}

void ImageFileWriter::_InitializeParameters()
{
    //this->declare_parameter("");
}

void ImageFileWriter::_UpdateParameters()
{
    //this->get_parameter("", );
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(image_file_writer::ImageFileWriter)

