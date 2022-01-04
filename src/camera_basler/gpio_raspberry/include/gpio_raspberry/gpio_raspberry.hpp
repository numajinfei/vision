/*
 * @Descripttion: 
 * @version: 
 * @Author: hw
 * @Date: 2021-04-30 10:49:55
 * @LastEditors: hw
 * @LastEditTime: 2021-10-18 23:55:53
 */
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

extern "C"
{
    #include "gpiod.h"
}

namespace gpio_raspberry
{

class GpioRaspberry : public rclcpp::Node
{
public:
    explicit GpioRaspberry(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~GpioRaspberry();

private:
    void _InitializeParameters();
    void _UpdateParameters();
    void _GpioGrabImage(void);
    void _GpioGrabImageOnce(void);

private:
    std::string _comsumer = "comsumer";
    struct gpiod_chip *_chip;
    struct gpiod_line *_lineBaslerTrigger;
    struct gpiod_line *_lineLaserTrigger;
    
    // int _port = 26;
    int _laser_line = 4;
    int _basler_trigger_line = 17;

    const char* _srvHighName = "~/high";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvHigh;

    const char* _srvLowName = "~/low";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvLow;

    const char* _srvToggleName = "~/toggle";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvToggle;

    // const char* _srvHighName = "~/camera_high";
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvHigh;

    // const char* _srvLowName = "~/camera_low";
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvLow;

    const char* _srvGrabStartName = "~/grab_start";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvGrabStart;

    const char* _srvGrabStopName = "~/grab_stop";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvGrabStop;

    const char* _srvGrabOnceName = "~/grab_once";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvGrabOnce;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _clientEnableSaveL;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _clientDisableSaveL;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _clientEnableSaveR;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _clientDisableSaveR;

    bool loop = false;
    std::thread _grab;

    int serial_number = 0;
};

}
