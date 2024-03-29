#include "mqtt_ros/mqtt_ros.hpp"

#include "mqtt/async_client.h"



namespace mqtt_ros
{
using namespace std::chrono_literals;

class MqttRos::_Impl
{
public:
    explicit _Impl(MqttRos* ptr) : _node(ptr), _callback(this)
    {
        _InitializeParameters();

        _UpdateParameters();

        _cli = std::make_shared<mqtt::async_client>(_serverAddress, _clientId, nullptr);
        _cli->set_callback(_callback);
        _cli->connect();	 
    }

    ~_Impl()
    {
        std::cout<<" ~_Impl()"<< std::endl;
        _cli->disconnect();
    }

    void PublishMqtt(const std::string& str)
    {
        std::size_t found = str.find("status");
        if(found != std::string::npos)
        {
            _cli->publish(_topic + "/status", str.c_str(), str.size(), _qos, true);
            return;
        }

        found = str.find("result");
        if(found != std::string::npos)
        {
            _cli->publish(_topic + "/result", str.c_str(), str.size(), _qos, false);
            std::cout<<"[mqtt_ros]: publish result successfully" <<std::endl;
            return;
        }

        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Unrecognized message in PublishMqtt!");
    }

    void PublishRos(const std::string& str)
    {
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = str;
        //std::cout << "msg is %s \n" << str;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[mqtt_ros] publish ros message: %s",str.c_str());
        _node->Publish(msg);
    }

    void Subscribe()
    {
        _cli->subscribe(_topic + "/control", _qos);
    }

    void SetConnectLostFlag(bool value)
    {
        _node->SetConnectLostFlag(value);
    }
    
    
    void MqttReconnect(void)
    {
        _cli->connect();
    }

private:
    void _InitializeParameters()
    {
        _node->declare_parameter("server_address");
        _node->declare_parameter("client_id");
        _node->declare_parameter("topic");
        _node->declare_parameter("qos");
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

    private:
        void connected(const std::string& /*cause*/) override
        {
            _impl->SetConnectLostFlag(false);
            _impl->Subscribe();
        }

        void connection_lost(const std::string& /*cause*/) override
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "MQTT connection lost!");
            _impl->SetConnectLostFlag(true);
        }

        void message_arrived(mqtt::const_message_ptr msg) override
        {
            _impl->PublishRos(msg->to_string());
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

MqttRos::MqttRos(const rclcpp::NodeOptions& options) : Node("mqtt_ros_node", options)
{
    _init = std::thread(&MqttRos::_Init, this);
}

MqttRos::~MqttRos()
{
    _loop = false;
    _init.join();

    _sub.reset();
    _impl.reset();
    _pub.reset();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "mqtt_ros destroyed successfully");
}

void MqttRos::Publish(std_msgs::msg::String::UniquePtr& ptr)
{
    _pub->publish(std::move(ptr));
    // RCLCPP_INFO(rclcpp::get_logger("mqtt"),"publish result successfully");//todo
}

void MqttRos::_Init() try
{

    std::this_thread::sleep_for(5000ms); //time delay to sure other nodes start up
    
    _pub = this->create_publisher<std_msgs::msg::String>(_pubName, 10);

    _impl = std::make_unique<_Impl>(this);

    _sub = this->create_subscription<std_msgs::msg::String>(_subName, 10, std::bind(&MqttRos::_Sub, this, std::placeholders::_1));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "mqtt_ros initialized successfully");
    
    _loop = true;
    
    int retry_count = 0;
    while(_loop)
    {
	    
	    if(_connect_lost_flag != false)
	    {
	      retry_count++;
	      if(retry_count >= 5)
	      {
		retry_count = 0;
		RCLCPP_INFO(rclcpp::get_logger("mqtt"),"mqtt node shut down");
		rclcpp::shutdown();		
		break;
	      }
	      else
	      {	
		RCLCPP_INFO(rclcpp::get_logger("mqtt"),"mqtt link reconnect");
		_impl->MqttReconnect();	                  
	      }
	    }
	    else
	    {	    		    	
	        if(retry_count != 0)
	    	    retry_count = 0;	    	
	    }

	    std::this_thread::sleep_for(5000ms);
     }
     
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in mqtt_ros initializer: %s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in mqtt_ros initializer: unknown");
    rclcpp::shutdown();
}

void MqttRos::_Sub(std_msgs::msg::String::UniquePtr ptr) try
{
    _impl->PublishMqtt(ptr->data);
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in mqtt_ros subscription: %s", e.what());
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in mqtt_ros subscription: unknown");
}

void MqttRos::SetConnectLostFlag(bool value)
{
    _connect_lost_flag = value;
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mqtt_ros::MqttRos)

