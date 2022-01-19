#include "script_json/script_json.hpp"

#include <nlohmann/json.hpp>

namespace script_json
{
using namespace std::chrono_literals;

using json = nlohmann::json;

bool EndsWith(const std::string& value, const std::string& ending)
{
    if(value.size() < ending.size())
        return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

void CheckAndSend(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ptr)
{
    if(ptr == nullptr)
        return;

    auto t = std::make_shared<std_srvs::srv::Trigger::Request>();

    if(rclcpp::ok() && ptr->service_is_ready())
        ptr->async_send_request(t);
    else
        throw std::runtime_error("Service failed");
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

ScriptJson::ScriptJson(const rclcpp::NodeOptions& options) : Node("script_json_node", options)
{
    _init = std::thread(&ScriptJson::_Init, this);
}

ScriptJson::~ScriptJson()
{
    _init.join();
    _sub.reset();
    if(_future.valid())
        _future.get();
    _pub.reset();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "script_json destroyed successfully");
}

void ScriptJson::_Init()
{
    _InitializeParameters();

    _UpdateParameters();

    while(true)
    {
        if(_IsNodesReady())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Nodes ready");
            break;
        }

        CheckAndSleep(2);
    }

    while(true)
    {
        if(_IsServicesReady())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Services ready");
            break;
        }

        CheckAndSleep(1);
    }

    for(auto& p : _map)
    {
        const auto& n = p.first;
        p.second = this->create_client<std_srvs::srv::Trigger>(n);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Services ready [%s]", n.c_str());
    }

    _paramClient = std::make_unique<rclcpp::AsyncParametersClient>(this, "/point_cloud_analyse_node");
    while(true)
    {
        if(_paramClient->service_is_ready())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parameter client ready");
            break;
        }

        CheckAndSleep(1);
    }

    _pub = this->create_publisher<std_msgs::msg::String>(_pubName, 10);

    _sub = this->create_subscription<std_msgs::msg::String>(_subName, 10, std::bind(&ScriptJson::_Sub, this, std::placeholders::_1));

    _subResult = this->create_subscription<shared_interfaces::msg::Float64Array>(_subResultName, 10, std::bind(&ScriptJson::_SubResult, this, std::placeholders::_1));

    _PublishStatus(STATUS_READY);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "script_json initialized successfully");
}

bool ScriptJson::_IsNodesReady()
{
    auto graph = this->get_node_names();
    for(const auto& e : _requiredNodes)
    {
        auto iter = std::find_if(graph.begin(), graph.end(), [&e](const std::string& g)
        {
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"g: %s, e: %s", g.c_str(), e.c_str());

            return EndsWith(g, e);
        });

        if(iter == graph.end())
            return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[_IsNodesReady] all needed nodes are ready");
    return true;
}

bool ScriptJson::_IsServicesReady()
{
    auto srvs = this->get_service_names_and_types();
    for(const auto& ele : _requiredServices)
    {
        const auto& n = ele.first;
        const auto& v = ele.second;

        for(const auto& s : v)
        {
            auto e = n + '/' + s;
            auto pos = std::find_if(srvs.begin(), srvs.end(), [&e](const std::pair<std::string, std::vector<std::string>>& p)
            {
                return EndsWith(p.first, e);
            });

            if(pos == srvs.end())
                return false;
            else
                _map[pos->first] = nullptr;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[_IsServicesReady] all needed services are ready");
    return true;
}

void ScriptJson::_Execute() try
{
    if(_param == "LineLaserOn")
    {
        CheckAndSend(_map["/gpio_raspberry_node/high"]);

        CheckAndSend(_map["/motor_encoder_node/center"]);

        CheckAndSleep(5);

        _PublishStatus(STATUS_READY);

        return;
    }

    if(_param == "LineLaserOff")
    {
        CheckAndSend(_map["/gpio_raspberry_node/low"]);

        CheckAndSend(_map["/motor_encoder_node/zero"]);

        CheckAndSleep(5);

        _PublishStatus(STATUS_READY);

        return;
    }

    if(_param == "GroundFlatness" || _param == "SideWallFlatness" || _param == "TopPlateFlatness")
    {
        _paramClient->set_parameters({rclcpp::Parameter("mode", 0)});
    }
    else if(_param == "ExternalCorner" || _param == "InsideCorner")
    {
        _paramClient->set_parameters({rclcpp::Parameter("mode", 1)});
    }
    else if(_param == "RoofLevel" || _param == "GroundLevel")
    {
        _paramClient->set_parameters({rclcpp::Parameter("mode", 2)});
    }
    else if(_param == "WallVertical")
    {
        _paramClient->set_parameters({rclcpp::Parameter("mode", 3)});
    }
    else if(_param == "StraightnessT")
    {
        _paramClient->set_parameters({rclcpp::Parameter("mode", 4)});
       

        CheckAndSend(_map["/camera_node/start"]);
        CheckAndSend(_map["/camera_node_r/start"]);
        CheckAndSend(_map["/motor_encoder_node/center"]);
        CheckAndSend(_map["/gpio_raspberry_node/high"]);
        CheckAndSleep(5);//=1S
        CheckAndSend(_map["/gpio_raspberry_node/grab_once"]);
        CheckAndSleep(5);//=1S
        CheckAndSend(_map["/camera_node/stop"]);    
        CheckAndSend(_map["/camera_node_r/stop"]);
        CheckAndSend(_map["/gpio_raspberry_node/low"]);
        CheckAndSend(_map["/motor_encoder_node/zero"]);
        CheckAndSleep(10);//=2S

        return;
    }
    else if( _param == "PlaneFlatnessT")
    {
        _paramClient->set_parameters({rclcpp::Parameter("mode", 5)});
    }
    else
    {
        _PublishStatus(STATUS_READY);
        throw std::runtime_error("Unsupported operation");
    }

	CheckAndSend(_map["/camera_node/start"]);
    CheckAndSend(_map["/camera_node_r/start"]);
    CheckAndSend(_map["/motor_encoder_node/zero"]);

    CheckAndSend(_map["/gpio_raspberry_node/high"]);

    CheckAndSleep(5);

    

    CheckAndSend(_map["/motor_encoder_node/scan"]);
	CheckAndSend(_map["/gpio_raspberry_node/grab_start"]);
	CheckAndSleep(20);//=4S

    CheckAndSend(_map["/gpio_raspberry_node/grab_stop"]); 
    CheckAndSleep(5);//=1S

    CheckAndSend(_map["/camera_node/stop"]);
    CheckAndSend(_map["/camera_node_r/stop"]);
    
    CheckAndSend(_map["/gpio_raspberry_node/low"]);
	CheckAndSend(_map["/motor_encoder_node/zero"]);
	CheckAndSleep(5); //=1s 

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "_Execute() is over....");
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in pipeline execute: %s", e.what());
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in pipeline execute: unknown");
}

void ScriptJson::_Sub(std_msgs::msg::String::UniquePtr ptr) try
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "_status is %d", _status);
    if(_future.valid() && _future.wait_for(1s) != std::future_status::ready)
    {
        _PublishResult(-1);
         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "thread is not ready....");
    }
    else{
        
        if(_status == STATUS_READY)
        {            
            auto j = json::parse(ptr->data);            
            _id = j["context"]["id"];            
            _param = j["context"]["params"][0];
            
            if(j["context"]["op"] == "measure")
            {		               		
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[script_json]: id:%d, param:%s", _id, _param.c_str());
                
                _future = std::async(std::launch::async, &ScriptJson::_Execute, this);
                _PublishStatus(STATUS_BUSY);
            }
            else
            {
                _PublishResult(-2);
            }
        }
        else
        {
            _PublishResult(-1);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[script_json]: ignore request");
        }
    }
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in script_json subscription: %s", e.what());
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in script_json subscription: unknown");
}

void ScriptJson::_SubResult(shared_interfaces::msg::Float64Array::UniquePtr ptr) try
{
    _PublishStatus(STATUS_READY);
    _PublishResult(0, ptr->data);    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "is ready....");
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in script_json subscription result: %s", e.what());
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in script_json subscription result: unknown");
}

void ScriptJson::_InitializeParameters()
{
    this->declare_parameter("required_nodes");
    this->get_parameter("required_nodes", _requiredNodes);

    for(const auto& n : _requiredNodes)
    {
        this->declare_parameter(n);
        std::vector<std::string> temp;
        this->get_parameter(n, temp);
        _requiredServices[n] = std::move(temp);
    }
}

void ScriptJson::_UpdateParameters()
{
}

void ScriptJson::_PublishStatus(STATUS status)
{
    json j;
    j["code"] = 0;
    switch(status)
    {
    case STATUS_READY:
        j["context"]["status"] = "ready";
        _status = STATUS_READY;
        break;
    case STATUS_BUSY:
        j["context"]["status"] = "busy";
		j["code"] = -1;
        _status = STATUS_BUSY;
        break;
    case STATUS_ERROR:
        j["context"]["status"] = "error";
		j["code"] = -3;
        _status = STATUS_ERROR;
        break;
    }

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = j.dump();
    _pub->publish(std::move(msg));
}

void ScriptJson::_PublishResult(int code, const std::vector<double>& result)
{
    json j;
    j["code"] = code;
    j["context"]["id"] = _id;
    j["context"]["result"] = json::array();

    json t;
    t["name"] = _param;
    t["value"] = 0.;

    if(_param == "TopPlateFlatness" || _param == "GroundFlatness" || _param == "SideWallFlatness")
    {
        t["optional"]["a"] = 0.;
        t["optional"]["b"] = 0.;
        t["optional"]["c"] = 0.;
        t["optional"]["d"] = 0.;
        t["optional"]["e"] = 0.;
        t["optional"]["f"] = 0.;
        t["optional"]["g"] = 0.;
    }

    if(code == 0)
    {
        if(result.empty())
        {
            j["code"] = -3;
        }
        else
        {
            t["value"] = result[0];
                        
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "result: %f, %f, %f, %f, %f, %f, %f, %f", 
              result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7]);
            
            if(_param == "TopPlateFlatness" || _param == "GroundFlatness" || _param == "SideWallFlatness" )
            {
                t["optional"]["a"] = result[1];
                t["optional"]["b"] = result[2];
                t["optional"]["c"] = result[3];
                t["optional"]["d"] = result[4];
                t["optional"]["e"] = result[5];
                t["optional"]["f"] = result[6];
                t["optional"]["g"] = result[7];
            }
        }
    }

    j["context"]["result"].push_back(t);

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = j.dump();
    _pub->publish(std::move(msg));
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(script_json::ScriptJson)

