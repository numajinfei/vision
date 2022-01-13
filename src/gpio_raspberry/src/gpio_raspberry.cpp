#include "gpio_raspberry/gpio_raspberry.hpp"

#include <fstream>
#include <chrono>

namespace gpio_raspberry
{
using namespace std::chrono_literals;

void High(struct gpiod_line * line)
{
    gpiod_line_set_value(line, 1);
}

void Low(struct gpiod_line * line)
{
    gpiod_line_set_value(line, 0);
}

void Toggle(struct gpiod_line * line)
{
    int val = gpiod_line_get_value(line);
    if(val)
        Low(line);
    else
        High(line);
}

void GpioRaspberry::_GpioGrabImage(void)
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>(); //临时创建shared[Trigger]指针 srv request，函数结束后自动释放 
  int num = 0;
  if(rclcpp::ok() && _clientDisableSaveL->service_is_ready() && _clientDisableSaveR->service_is_ready())
  {
    _clientDisableSaveL->async_send_request(request);
    _clientDisableSaveR->async_send_request(request);

    std::this_thread::sleep_for(5ms);

    while(loop && (num < 300)) //TODO： 300 待确定单次测量最大所需图片数
    {        
      if(rclcpp::ok())
      {
      	
        std::this_thread::sleep_for(16ms); //=1000/帧率
        Toggle(_lineBaslerTrigger);
        //std::cout<<"toggle"<<std::endl;
        num ++;
      }            
      else
        throw std::runtime_error("Interrupted!");
    }
    
    if(num >= 300)
    {
    	RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "GrabImage' num is more than 300!!!");
    }
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GrabImage' num is %d", num);
    num = 0;
  }
}

void GpioRaspberry::_GpioGrabImageOnce(void)
{

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>(); //临时创建shared[Trigger]指针 srv request，函数结束后自动释放 

  if(rclcpp::ok() && _clientEnableSaveL->service_is_ready() && _clientEnableSaveR->service_is_ready())
  {
    _clientEnableSaveL->async_send_request(request);
    _clientEnableSaveR->async_send_request(request);

    std::this_thread::sleep_for(5ms);

    Toggle(_lineBaslerTrigger);

  }
}

GpioRaspberry::GpioRaspberry(const rclcpp::NodeOptions& options) : Node("gpio_raspberry_node", options)
{
    _InitializeParameters();

    _UpdateParameters();

    _srvHigh = this->create_service<std_srvs::srv::Trigger>(_srvHighName, [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try
        {
            response->success = false;
            response->message = "Failed: IO set to high";

            High(_lineLaserTrigger);
            response->success = true;
            response->message = "Success: IO set to high";
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service high: %s", e.what());
        }
        catch(...)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service high: unknown");
        }
    });

    _srvLow = this->create_service<std_srvs::srv::Trigger>(_srvLowName, [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try
        {
            response->success = false;
            response->message = "Failed: IO set to low";

            Low(_lineLaserTrigger);
            response->success = true;
            response->message = "Success: IO set to low";
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service low: %s", e.what());
        }
        catch(...)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service low: unknown");
        }
    });

    _srvToggle = this->create_service<std_srvs::srv::Trigger>(_srvToggleName, [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try
        {
            response->success = false;
            response->message = "Failed: IO toggle";

            Toggle(_lineLaserTrigger);
            response->success = true;
            response->message = "Success: IO toggle";
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service toggle: %s", e.what());
        }
        catch(...)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service toggle: unknown");
        }
    });

    _srvGrabStart = this->create_service<std_srvs::srv::Trigger>(_srvGrabStartName, [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try
        {
            response->success = false;
            response->message = "Failed: Grab Started";

            loop = true;
            _grab = std::thread(&GpioRaspberry::_GpioGrabImage, this);
            _grab.detach();
                        
            response->success = true;
            response->message = "Success: Grab Started";
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[_srvGrabStart-1]: %s", response->message);
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service toggle: %s", e.what());
        }
        catch(...)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service toggle: unknown");
        }
    });

    _srvGrabStop = this->create_service<std_srvs::srv::Trigger>(_srvGrabStopName, [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try
        {
            response->success = false;
            response->message = "Failed: Grab Stop";

            loop = false;
                                    
            response->success = true;
            response->message = "Success: Grab Stop";
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[_srvGrabStart-2]: %s", response->message);
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service toggle: %s", e.what());
        }
        catch(...)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service toggle: unknown");
        }
    });

    _srvGrabOnce = this->create_service<std_srvs::srv::Trigger>(_srvGrabOnceName, [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try
        {
            response->success = false;
            response->message = "Failed: Grab once";

            _GpioGrabImageOnce();
                                    
            response->success = true;
            response->message = "Success: Grab once";
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service toggle: %s", e.what());
        }
        catch(...)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service toggle: unknown");
        }
    });

    _clientEnableSaveL = this -> create_client<std_srvs::srv::Trigger>("/camera_node/enableSave");
    _clientDisableSaveL = this -> create_client<std_srvs::srv::Trigger>("/camera_node/disableSave");

    _clientEnableSaveR = this -> create_client<std_srvs::srv::Trigger>("/camera_node_r/enableSave");
    _clientDisableSaveR = this -> create_client<std_srvs::srv::Trigger>("/camera_node_r/disableSave");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gpio_raspberry initialized successfully");
}

GpioRaspberry::~GpioRaspberry() try
{
    _srvHigh.reset();
    _srvLow.reset();
    _srvToggle.reset();
    _srvGrabStart.reset();
    _srvGrabStop.reset();
    _srvGrabOnce.reset();
    _clientEnableSaveL.reset();
    _clientEnableSaveL.reset();
    _clientEnableSaveR.reset();
    _clientEnableSaveR.reset();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gpio_raspberry destroyed successfully");
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in destructor: %s", e.what());
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in destructor: unknown");
}

void GpioRaspberry::_InitializeParameters()
{
    this->declare_parameter("port");
    this->declare_parameter("basler_trigger_line");
    this->declare_parameter("laser_trigger_line");
    
    this->get_parameter("basler_trigger_line", _basler_trigger_line);
    this->get_parameter("laser_trigger_line", _laser_line);
    

    std::string chipname("gpiochip0");
    // Open GPIO chip
    _chip = gpiod_chip_open_by_name(chipname.c_str());
    if(!_chip)
        throw std::runtime_error("gpio chip" + chipname + "open failed!");

    // Open GPIO lines
    _lineBaslerTrigger = gpiod_chip_get_line(_chip, _basler_trigger_line); //gpio17
    if(!_lineBaslerTrigger)
        throw std::runtime_error("Open GPIO line failed!");        
    // Open Trriger lines for output
    gpiod_line_request_output(_lineBaslerTrigger, _comsumer.c_str(), 0);

     _lineLaserTrigger = gpiod_chip_get_line(_chip, _laser_line); //gpio7
    if(!_lineLaserTrigger)
        throw std::runtime_error("Open GPIO line failed!");
    gpiod_line_request_output(_lineLaserTrigger, _comsumer.c_str(), 0);
}

void GpioRaspberry::_UpdateParameters()
{
    // this->get_parameter("port", _port);
    this->get_parameter("basler_trigger_line", _basler_trigger_line);
    this->get_parameter("laser_trigger_line", _laser_line);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gpio_raspberry::GpioRaspberry)
