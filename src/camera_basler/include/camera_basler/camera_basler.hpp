/*
 * @Descripttion: 
 * @version: 
 * @Author: hw
 * @Date: 2021-09-13 16:32:24
 * @LastEditors: hw
 * @LastEditTime: 2022-01-11 14:53:49
 */
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace camera_basler
{
class CameraBasler : public rclcpp::Node
{
public:
    explicit CameraBasler(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CameraBasler();

    // void Publish(sensor_msgs::msg::Image::UniquePtr& ptr)
    // {  
    //     _pub->publish(std::move(ptr));
    //     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"success to pub");    
    //     std::cout<<"success to pub.\n"<<std::flush;    
    // }
    
    // void PublishL(sensor_msgs::msg::Image::UniquePtr& ptr)
    // {  
    //     _pubL->publish(std::move(ptr));
    //     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"success to pubL");    
    //     std::cout<<"success to pubL.\n"<<std::flush;    
    // }

    void PublishR(sensor_msgs::msg::Image::UniquePtr& ptr)
    {
        _pubR->publish(std::move(ptr));
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"success to pubR");   
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"node: %s  getID: %s", this -> get_name(), ptr->header.frame_id.c_str() );
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"getID: %s", ptr->header.frame_id );
        // std::cout<<"success to pubR.\n"<<std::flush;    
    }

    std::string _GetCameraSerialNumber();
    std::string _GetSavePath();
    // std::string _GetCamera2SerialNumber();

    void SetSaveImage(bool status = false)
    {
        saveImage = status;
    }

    bool GetSaveImage()
    {
        return saveImage;
    }

    std::string GetFrameID()
    {
        return std::to_string(++frameID);
    }

    std::string GetFrameIDStatic()
    {
        return std::to_string(frameID);
    }

    void ResetFrameID()
    {
        frameID = 0;
    }

private:
    void _Init();
    void _Start(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void _Stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void _EnableSave(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void _DisableSave(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);  

private:
    class _Impl;
    std::unique_ptr<_Impl> _impl;

    //OnSetParametersCallbackHandle::SharedPtr _callback;

    // const char* _pubLName = "~/image_l";
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pubL;

    const char* _pubRName = "~/image";
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pubR;    

    const char* _srvStartName = "~/start";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvStart;

    const char* _srvStopName = "~/stop";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvStop;

    const char* _srvEnableSaveName = "~/enableSave";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvEnableSave;

    const char* _srvDisableSaveName = "~/disableSave";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvDisableSave;

    std::thread _init;

    bool saveImage = false;

    int frameID = 0;
};
}