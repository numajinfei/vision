#include "condition_monitoring/condition_monitoring.hpp"

#include "nlohmann/json.hpp"

namespace condition_monitoring
{
using namespace std::chrono_literals;

using json = nlohmann::json;

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

bool EndsWith(const std::string& value, const std::string& ending)
{
    if(value.size() < ending.size())
        return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

void CheckAndSleep(int count)
{
    for(int i = 0; i < count; i++)
    {
        if(rclcpp::ok())
            std::this_thread::sleep_for(200ms);
        else
            throw std::runtime_error("Interrupted!");
    }
}

class ConditionMonitoring::_Impl
{
    public:
        explicit  _Impl(ConditionMonitoring* ptr) : _node(ptr)
        {
            _Init();
        }

        ~_Impl()
        {

        }

        void _Init() try
        {
            _InitializeParameters();

            _UpdateParameters();
                
            int numbers = 0;

            while(true)
            {
                if(_IsNodesReady())
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Nodes ready");
                    numbers = 0;
                    break;
                }

                CheckAndSleep(1);
                if(++numbers > 25)
                {
                    RCLCPP_INFO(_node->get_logger(), "Waiting for all nodes ready");
                    numbers = 0;
                }
            }

            while(true)
            {
                if(_IsServicesReady())
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Services ready");
                    numbers = 0;
                    break;
                }

                CheckAndSleep(1);
                if(++numbers > 25)
                {
                    RCLCPP_INFO(_node->get_logger(), "Waiting for all services ready");
                    numbers = 0;
                }
            }

            // _PubInitStatus();

            return;
        }
        catch(std::exception& e)
        {
            RCLCPP_INFO(_node->get_logger(),"Error in _impl(): %s", e.what());
            rclcpp::shutdown();
        }
        catch(...)
        {
            RCLCPP_INFO(_node->get_logger(),"Error in _impl(): unknown.");
            rclcpp::shutdown();           
        }

        void _InitializeParameters()
        {
            _node->declare_parameter<std::vector<std::string>>("required_nodes",{""});
            _node->get_parameter("required_nodes", _requiredNodes);

            for(const auto& node : _requiredNodes)
            {
                _node->declare_parameter<std::vector<std::string>>(node,{""});
                std::vector<std::string> temp;
                _node->get_parameter(node, temp);
                _requiredServices[node] = std::move(temp);
            }

            // _node->declare_parameter<std::vector<std::string>>("aichitecture_measurement_nodes",{});
            // _node->get_parameter("aichitecture_measurement_nodes", _architectureMeasurementNodes);

            // _node->declare_parameter<bool>("isTest", false);
            // _node->get_parameter("isTest", isTest);
            return;
        }

        void _UpdateParameters()
        {
            return;
        }

        bool _IsNodesReady()
        {
            auto node_names = _node->get_node_names();
            for(const auto& node : _requiredNodes)
            {
                auto iter = std::find_if(node_names.begin(), node_names.end(), [&node](const std::string& name)
                {
                    return EndsWith(name, node);
                });

                if(iter == node_names.end())
                {
                    std::cout << node << " dose not been found." << std::endl;
                    return false;
                }
                // else
                //     std::cout << "has been found." << std::endl;
                    
            }
            return true;
        }

        bool _IsServicesReady()
        {
            auto srv_names = _node->get_service_names_and_types();
            for(const auto& srv : _requiredServices)
            {
                const auto& srv_first = srv.first;
                const auto& srv_second = srv.second;

                for(const auto& iter : srv_second)
                {
                    if(iter == "")
                        continue;
                    auto srv_full_name = srv_first + '/' + iter;
                    std::cout << "service: " << srv_full_name << " ";
                    auto pos = std::find_if(srv_names.begin(), srv_names.end(), [&srv_full_name](const std::pair<std::string, std::vector<std::string>>& pair)
                    {
                        return EndsWith(pair.first, srv_full_name);
                    });

                    if(pos == srv_names.end())
                    {
                        std::cout << "dose not been found." << std::endl;
                        return false;
                    }
                    // else
                    //     std::cout << "has been found." << std::endl;   
                }
            }
            return true;
        }

        void _PubInitStatus()
        {
            json j;
            j["code"] = "0";
            j["context"]["status"] = "idle";
            j["context"]["camera_status"] = "idle";
            j["context"]["inclinationery_status"] = "idle";
            j["context"]["laser_range_status"] = "idle";

            std::string status = j.dump();
            _node->_PubStatus(status);
        }

        // void _PubStatus(const std::string& status)
        // {
        //     // auto json_parse = json::parse(status);

        //     // std::string _module = json_parse["context"]["module"];
        //     // std::string _status = json_parse["context"]["status"];
        //     // std::string _statusNum = json_parse["context"]["statusNum"];

        //     // json j;
        //     // j["code"] = 0;
        //     // j["context"]["status"] = "idle";
        //     // j["context"]["camera_status"] = "idle";
        //     // j["context"]["inclinationery_status"] = "idle";
        //     // j["context"]["laser_range_status"] = "idle";

        //     // auto iter = std::find_if(_architectureMeasurementNodes.begin(), _architectureMeasurementNodes.end(), [&_module](const std::string& name)
        //     // {
        //     //     return EndsWith(name, _module);
        //     // });
        //     // if(iter < _architectureMeasurementNodes.end())
        //     // {
        //     //     if(_status == "error")
        //     //         j["context"]["status"] = "failure";
        //     // }
        //     // else if(_module == "photoneo_node")
        //     // {
        //     //     if(_status == "error")
        //     //     {
        //     //         j["context"]["status"] = "failure";
        //     //         j["context"]["camera_status"] = "failure";
        //     //     }
        //     // }   
        //     // else if(_module == "inclinometer_node")
        //     // {
        //     //     if(_status == "error")
        //     //     {
        //     //         j["context"]["status"] = "failure";
        //     //         j["context"]["inclinationery_status"] = "failure";
        //     //     }
        //     // } 
        //     // else if(_module == "laser_ranger_node")
        //     // {
        //     //     if(_status == "error")
        //     //     {
        //     //         j["context"]["status"] = "failure";
        //     //         j["context"]["laser_range_status"] = "failure";
        //     //     }
        //     // }  
        //     // else if(_module == "function_node")    
        //     // {
        //     //     if(_status == "error")
        //     //         j["context"]["status"] = "failure";
        //     // }
        //     // else
        //     // {
        //     //     throw std::runtime_error("Error: _module is invalid.");
        //     // }

        //     // std::string pub_status = j.dump();

        //     _node->_PubStatus(status);
        //     RCLCPP_INFO(_node->get_logger(), "%s publish initialize status: %s", GetLocalTime(), pub_status.c_str());            
        // }

    private:
        std::vector<std::string> _requiredNodes;

        std::map<std::string, std::vector<std::string>> _requiredServices;

        std::vector<std::string> _architectureMeasurementNodes;

        ConditionMonitoring* _node;        
};

/**** ConditionMonitoring ****/
ConditionMonitoring::ConditionMonitoring(const rclcpp::NodeOptions& options) : Node("condition_monitoring_node", options)
{
    _initThread = std::thread(&ConditionMonitoring::_Init, this);
}

ConditionMonitoring::~ConditionMonitoring() try
{
    _initThread.join();

    _impl.reset();
    _pubStatus.reset();
    _subStatus.reset();

    RCLCPP_INFO(this->get_logger(),"ConditionMonitoring destroyed successfully");
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(this->get_logger(), "Exception in ConditionMonitoring initializer: %s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(this->get_logger(),"Exception in ConditionMonitoring initializer: unknown");
    rclcpp::shutdown();
}

void ConditionMonitoring::_Init() try
{
    _status = -1;    

    _InitializeParameters();

    _UpdateParameters();

    _impl = std::make_unique<_Impl>(this);

    _pubStatus = this->create_publisher<std_msgs::msg::String>(_pubStatusName, 10);

    _subStatus = this->create_subscription<std_msgs::msg::String>(_subStatusName, 10, std::bind(&ConditionMonitoring::_SubStatus, this, std::placeholders::_1));

    _status = 0;    

    RCLCPP_INFO(this->get_logger(), "ConditionMonitoring initialized successfully");

    _impl->_PubInitStatus();
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(this->get_logger(), "Exception in ConditionMonitoring initializer: %s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(this->get_logger(),"Exception in ConditionMonitoring initializer: unknown");
    rclcpp::shutdown();
}

void ConditionMonitoring::_InitializeParameters()
{
    return;
}

void ConditionMonitoring::_UpdateParameters()
{
    return;
}

void ConditionMonitoring::_SubStatus(std_msgs::msg::String::UniquePtr ptr)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "%s Error in _SubStatus(): initialize and receive a status: %s", GetLocalTime(), ptr->data.c_str());
        return;
    }
    _PubStatus(ptr->data);
}

void ConditionMonitoring::_PubStatus(const std::string& status)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "%s Error in _PubStatus(): initialize and publish a status: %s", GetLocalTime(), status.c_str());
        return;
    }
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = status;
    _pubStatus->publish(std::move(msg));
}

}//namespace condition_monitoring

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(condition_monitoring::ConditionMonitoring)

