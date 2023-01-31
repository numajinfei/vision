#include "mqtt_ros/mqtt_ros.hpp"

#include "mqtt/async_client.h"

namespace mqtt_ros
{

const char* GetLocalTime()
{
    time_t now = time(0);
    std::tm* localTime = std::localtime(&now);
    static const std::string time = "[" + std::to_string(1900 + localTime->tm_year) + "-" 
            + std::to_string(1 + localTime->tm_mon) + "-"
            + std::to_string(localTime->tm_mday) + " "
            + std::to_string(localTime->tm_hour) + ":"
            + std::to_string(localTime->tm_min) + ":"
            + std::to_string(localTime->tm_sec) + "]";
    static const char* time_ptr = time.c_str();
    return time_ptr;
}

class MqttRos::_Impl
{
public:
    explicit _Impl(MqttRos* ptr) : _node(ptr), _callback(this) 
    {
        try
        {
            _InitializeParameters();

            _UpdateParameters();

            _cli = std::make_shared<mqtt::async_client>(_serverAddress, _clientId, nullptr);
            _cli->set_callback(_callback);
            _cli->connect();
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO(_node->get_logger(), "Error in _Impl(): %s", e.what());
            rclcpp::shutdown();
        }
        catch(...)
        {
            RCLCPP_INFO(_node->get_logger(), "Error in _Impl(): unknown.");
            rclcpp::shutdown();
        }
    }


    ~_Impl() try
    {
        _cli->disconnect();
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(_node->get_logger(), "Error in _Impl(): %s", e.what());
        rclcpp::shutdown();
    }
    catch(...)
    {
        RCLCPP_INFO(_node->get_logger(), "Error in _Impl(): unknown.");
        rclcpp::shutdown();
    }


    void PublishResultToMqtt(const std::string& str)
    {
        std::size_t found = str.find("result");
        if(found != std::string::npos)
        {
            _cli->publish(_topic + "/result", str.c_str(), str.size(), _qos, false);
            RCLCPP_INFO(_node->get_logger(),"publish a result to mqtt: %s", str.c_str());
            return;
        }

        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Unrecognized message in PublishResultToMqtt!");
    }

    void PublishStatusToMqtt(const std::string& str)
    {
        std::size_t found = str.find("status");
        if(found != std::string::npos)
        {
            _cli->publish(_topic + "/status", str.c_str(), str.size(), _qos, true);
            RCLCPP_INFO(_node->get_logger(),"publish a status to mqtt: %s", str.c_str());
            return;
        }

        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Unrecognized message in PublishStatusToMqtt!");
    }

    void PublishMeasurementRequestToRos(const std::string& str)
    {
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = str;        
        _node->_PubMeasurementRequest(msg);
        RCLCPP_INFO(_node->get_logger(),"%s publish ros message: %s", GetLocalTime(), str.c_str());
    }

    void Subscribe()
    {
        _cli->subscribe(_topic + "/control", _qos);
    }

private:
    void _InitializeParameters()
    {
        _node->declare_parameter<std::string>("server_address","tcp://127.0.0.1:1883");
        _node->declare_parameter<std::string>("client_id","1");
        _node->declare_parameter<std::string>("topic","iot/zhushi/scanner/");
        _node->declare_parameter<int>("qos",0);
    }

    void _UpdateParameters()
    {
        _node->get_parameter("server_address", _serverAddress);
        _node->get_parameter("client_id", _clientId);
        _node->get_parameter("topic", _topic);			
        _node->get_parameter("qos", _qos);
		
		_topic += _clientId;
    }

    class Callback : public mqtt::callback
    {
        public:
            Callback(_Impl* ptr) : _impl(ptr)
            {

            }

            ~Callback()
            {

            }

        private:
            void connected(const std::string& /*cause*/) override
            {
                _impl->Subscribe();
            }

            void connection_lost(const std::string& /*cause*/) override
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "MQTT connection lost!");
                _impl->_cli->connect();
                // rclcpp::shutdown();
            }

            void message_arrived(mqtt::const_message_ptr msg) override
            {
                _impl->PublishMeasurementRequestToRos(msg->to_string());
            }

            void delivery_complete(mqtt::delivery_token_ptr /*tok*/) override
            {

            }

        private:
            _Impl* _impl;
    };

private:   
    std::string _serverAddress = "tcp://127.0.0.1:1883";
    std::string _clientId = "paho_cpp_async_subcribe";
    std::string _topic = "iot/zhushi/scanner/1";
    int _qos = 2;

    MqttRos* _node;
    Callback _callback;
    mqtt::async_client::ptr_t _cli;
};

/**** MqttRos ****/
MqttRos::MqttRos(const rclcpp::NodeOptions& options) : Node("mqtt_ros_node", options)
{
    _initThread = std::thread(&MqttRos::_Init, this);
}

MqttRos::~MqttRos() try
{
    _initThread.join();

    _impl.reset();
    _pubMeasurementRequest.reset();
    _subResult.reset();
    _subStatus.reset();

    RCLCPP_INFO(this->get_logger(),"MqttRos destroyed successfully");
}
catch(const std::exception& e)
{
    RCLCPP_INFO(this->get_logger(), "Exception in ~MqttRos(): %s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_INFO(this->get_logger(), "Exception in ~MqttRos(): unknown");
    rclcpp::shutdown();  
}

void MqttRos::_Init() try
{
    _status = -1;

    _impl = std::make_unique<_Impl>(this);

    _pubMeasurementRequest = this->create_publisher<std_msgs::msg::String>(_pubMeasurementRequestName, 10);
    _subResult = this->create_subscription<std_msgs::msg::String>(_subResultName, 10, std::bind(&MqttRos::_SubResult, this, std::placeholders::_1));
    _subStatus = this->create_subscription<std_msgs::msg::String>(_subStatusName, 10, std::bind(&MqttRos::_SubStatus, this, std::placeholders::_1));

    _status = 0;
    RCLCPP_INFO(this->get_logger(), "MqttRos initialized successfully");
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(this->get_logger(), "Exception in MqttRos initializer: %s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(this->get_logger(),"Exception in MqttRos initializer: unknown");
    rclcpp::shutdown();
}

void MqttRos::_InitializeParameters()
{
    return;
}

void MqttRos::_UpdateParameters()
{
    return;
}

void MqttRos::_PubMeasurementRequest(std_msgs::msg::String::UniquePtr& ptr)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"initialize and publish a measurement request: %s", ptr->data.c_str());
        return;
    }

    _pubMeasurementRequest->publish(std::move(ptr));

    return;
}

void MqttRos::_SubResult(std_msgs::msg::String::UniquePtr ptr)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"initialize and subscribe a result: %s", ptr->data.c_str());
        return;
    }

    _impl->PublishResultToMqtt(ptr->data);

    return;
}

void MqttRos::_SubStatus(std_msgs::msg::String::UniquePtr ptr)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"initialize and subscribe a status: %s", ptr->data.c_str());
        return;
    }

    // _impl->PublishStatusToMqtt(ptr->data);

    return;
}

}//namespace mqtt_ros

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mqtt_ros::MqttRos)

