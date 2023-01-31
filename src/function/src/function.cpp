#include "function/function.hpp"

namespace function
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

bool CompSize(std::vector<double>& v1, std::vector<double>& v2)
{
    return v1.size() > v2.size();
}

bool EndsWith(const std::string& value, const std::string& ending)
{
    if(value.size() < ending.size())
        return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}


void CheckAndSend(
    rclcpp::Client<shared_interfaces::srv::TriggerOp>::SharedPtr ptr, 
    const long& id,
    const std::string& engineering, 
    const std::string& option,
    const float& angle, 
    const int& frameID)
{
    auto triggerOp = std::make_shared<shared_interfaces::srv::TriggerOp::Request>();
    triggerOp->id = id;
    triggerOp->engineering = engineering;
    triggerOp->option = option;
    triggerOp->angle = angle;
    triggerOp->frame_id = frameID;

    if(rclcpp::ok() && ptr->service_is_ready())
        ptr->async_send_request(triggerOp);
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

std::unordered_map<float, int>  LUTWindowAngle()
{
    std::unordered_map<float, int> lookUpTable;
    lookUpTable.emplace(20, 0);
    lookUpTable.emplace(10, 1);
    lookUpTable.emplace(-10, 2);
    lookUpTable.emplace(-20, -1);

    return lookUpTable;
}

std::unordered_map<float, int>  LUTDoorAngle()
{
    std::unordered_map<float, int> lookUpTable;
    lookUpTable.emplace(30, 0);
    lookUpTable.emplace(10, 1);
    lookUpTable.emplace(-10, 2);
    lookUpTable.emplace(-30, -1);

    return lookUpTable;
}

class Function::_Impl
{
    public:
        explicit  _Impl(Function* ptr) : _node(ptr)
        {
            _Init();
        }

        ~_Impl()
        {

        }

        void _Init()
        {
            _InitializeParameters();

            _UpdateParameters();

            int numbers = 0;

            while(true)
            {
                if(_IsNodesReady())
                {
                    RCLCPP_INFO(_node->get_logger(), "Nodes ready");
                    numbers = 0;
                    break;
                }

                CheckAndSleep(1);
                if(++numbers > 25)
                {
                    RCLCPP_INFO(_node->get_logger(), "Waiting for nodes");
                    numbers = 0;
                }  
            }

            while(true)
            {
                if(_IsServicesReady())
                {
                    RCLCPP_INFO(_node->get_logger(), "Services ready");
                    numbers = 0;
                    break;
                }

                CheckAndSleep(1);
                if(++numbers > 25)
                {
                    RCLCPP_INFO(_node->get_logger(), "Waiting for services");
                    numbers = 0;
                }
            }

            for(auto& p : _map)
            {
                const auto& p_first = p.first;
                p.second = _node->create_client<shared_interfaces::srv::TriggerOp>(p_first);
                RCLCPP_INFO(_node->get_logger(), "Services ready [%s]", p_first.c_str());
            }

            return;
        }

        void _InitializeParameters()
        {
            _node->declare_parameter<std::vector<std::string>>("required_nodes",{"node"});
            _node->get_parameter("required_nodes", _requiredNodes);

            for(const auto& node : _requiredNodes)
            {
                _node->declare_parameter<std::vector<std::string>>(node,{""});
                std::vector<std::string> temp;
                _node->get_parameter(node, temp);
                _requiredServices[node] = std::move(temp);
            }

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
                    std::cout << "service: " << srv_full_name << std::endl;
                    auto pos = std::find_if(srv_names.begin(), srv_names.end(), [&srv_full_name](const std::pair<std::string, std::vector<std::string>>& pair)
                    {
                        return EndsWith(pair.first, srv_full_name);
                    });

                    if(pos == srv_names.end())
                    {
                        std::cout << " dose not been found." << std::endl;
                        return false;
                    }
                        
                    else
                        _map[pos->first] = nullptr;
                }
            }

            return true;
        }

        void _Execute()
        {
            if(_node->_value == "StoreyHeight") 
                CheckAndSend(_map["/aux_box_client_node/grabLaserRanger"], 0, std::string(), std::string(), 0, -1);   

            else if(_node->_value == "SideWallFlatness" || _node->_value == "TopPlateFlatness" || _node->_value == "GroundFlatness")
                CheckAndSend(_map["/phoxi_control_node/grab_point_cloud"], _node->_id, _node->_engineering, _node->_value, _node->_angle, _node->_frameID);

            else if(_node->_value == "WallVertical" || _node->_value == "RoofLevel" || _node->_value == "GroundLevel" || _node->_value == "PillarVerticality")
                CheckAndSend(_map["/phoxi_control_node/grab_point_cloud"], _node->_id, _node->_engineering, _node->_value, _node->_angle,_node->_frameID);

            else if(_node->_value == "InsideCorner" || _node->_value == "ExternalCorner" ) 
                CheckAndSend(_map["/phoxi_control_node/grab_point_cloud"], _node->_id, _node->_engineering, _node->_value, _node->_angle,_node->_frameID);

            else if(_node->_value == "PillarSectionSize")
                CheckAndSend(_map["/phoxi_control_node/grab_point_cloud"], _node->_id, _node->_engineering, _node->_value, _node->_angle,_node->_frameID);

            else if(_node->_value == "WindowOpeningSize" || _node->_value == "DoorOpeningSize")
            {
                RCLCPP_INFO(_node->get_logger(),"send request");
                CheckAndSend(_map["/phoxi_control_node/grab_point_cloud"], _node->_id, _node->_engineering, _node->_value, _node->_angle,_node->_frameID);
            }


            else if(_node->_value == "DefectDetection")
                CheckAndSend(_map["/phoxi_control_node/grab_image"], _node->_id, _node->_engineering, _node->_value, _node->_angle,_node->_frameID);

            else if(_node->_value == "EmbeddedParts")
                CheckAndSend(_map["/phoxi_control_node/grab_image"], _node->_id, _node->_engineering, _node->_value, _node->_angle,_node->_frameID);

            else
            {
                RCLCPP_INFO(_node->get_logger(), "%s Error in _Execute(): wrong measurement_item.", GetLocalTime());                              
            }

            return;
        }
    private:
        std::vector<std::string> _requiredNodes;
        std::map<std::string, std::vector<std::string>> _requiredServices;
        std::map<std::string, rclcpp::Client<shared_interfaces::srv::TriggerOp>::SharedPtr> _map;

        Function* _node;        
};

/**** Function ****/
Function::Function(const rclcpp::NodeOptions& options  = rclcpp::NodeOptions()) : Node("function_node", options)
{
    _initThread = std::thread(&Function::_Init, this);
}

Function::~Function() try
{
    _initThread.join();

    _impl.reset();
    _pubResult.reset();
    _pubStatus.reset();
    _subRequest.reset();
    _subResult.reset();
    _subResultEx.reset();
    _subResultDefect.reset();
    _subResultEmbeddedParts.reset();
    if(_future.valid())
        _future.get();

    RCLCPP_INFO(this->get_logger(),"Function destroyed successfully");
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(this->get_logger(), "Exception in Function initializer: %s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(this->get_logger(),"Exception in Function initializer: unknown");
    rclcpp::shutdown();
}

void Function::_Init() try
{
    _status = -1;

    _impl = std::make_unique<_Impl>(this);

    _InitializeParameters();

    _UpdateParameters();

    _pubResult = this->create_publisher<std_msgs::msg::String>(_pubResultName, 10);

    _pubStatus = this->create_publisher<std_msgs::msg::String>(_pubStatusName, 10);

    _subResult = this->create_subscription<shared_interfaces::msg::Float64Array>(_subResultName, 10, std::bind(&Function::_SubResult, this, std::placeholders::_1));  

    _subResultEx = this->create_subscription<shared_interfaces::msg::MeasurementResult>(_subResultNameEx, 10, std::bind(&Function::_SubResultEx, this, std::placeholders::_1));  

    _subResultDefect = this->create_subscription<shared_interfaces::msg::Defect>(_subResultDefectName,10, std::bind(&Function::_SubResultDefect,this,std::placeholders::_1));

    _subResultEmbeddedParts = this->create_subscription<shared_interfaces::msg::EmbeddedParts>(_subResultEmbeddedPartsName,10,std::bind(&Function::_SubResultEmbeddedParts,this,std::placeholders::_1));

    _subRequest = this->create_subscription<std_msgs::msg::String>(_subRequestName, 10, std::bind(&Function::_SubRequest, this, std::placeholders::_1));      

    _status = 0;

    RCLCPP_INFO(this->get_logger(), "Function initialized successfully");
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(this->get_logger(), "Exception in Function initializer: %s", e.what());
}
catch(...)
{
    RCLCPP_ERROR(this->get_logger(),"Exception in Function initializer: unknown");
}

void Function::_InitializeParameters()
{
    return;
}

void Function::_UpdateParameters()
{
    return;
}

void Function::_Execute()
{
    _impl->_Execute();
    return;
}

void Function::_SubRequest(std_msgs::msg::String::UniquePtr ptr)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "%s Error in _SubRequest(): initialize and subscribe a request.",GetLocalTime());
        return;
    }

    auto json_parse = json::parse(ptr->data);
    _id = json_parse["context"]["id"];
    _engineering = json_parse["context"]["engineering"];
    std::string angleTmp = json_parse["context"]["angle"];
    _angle = std::stof(angleTmp);
    _value = json_parse["context"]["value"];

    std::cout << "_id: " << _id << std::endl;
    std::cout << "_engineering: " << _engineering << std::endl;
    std::cout << "_angle: " << _angle << std::endl;
    std::cout << "_value: " << _value << std::endl;

    if(_value == "WindowOpeningSize")
    {
        std::cout << "window" << std::endl;
        std::unordered_map<float, int> umapWindow = LUTWindowAngle();
        std::unordered_map<float, int>::const_iterator iterWindow = umapWindow.find(_angle);
        if(iterWindow == umapWindow.end())
        {
            std::cout << "angle is invalid" << std::endl;
            return;
        }
        _frameID = iterWindow->second;
        std::cout << "_frameID: " << _frameID <<std::endl;
    }
    else if(_value == "DoorOpeningSize")
    {
        std::cout << "door" << std::endl;
        std::unordered_map<float, int> umapDoor = LUTDoorAngle();
        std::unordered_map<float, int>::const_iterator iterDoor = umapDoor.find(_angle);
        if(iterDoor == umapDoor.end())
        {
            std::cout << "angle is invalid" << std::endl;
            return;
        }
        _frameID = iterDoor->second;
        std::cout << "_frameID: " << _frameID <<std::endl;
    }
    else
    {
        _frameID = 0;
    }

    if(_future.valid() && _future.wait_for(0s) != std::future_status::ready)
        _PubStatus(1);
    else if(json_parse["context"]["op"] == "measure")
        _future = std::async(std::launch::async, &Function::_Execute, this);
    else
        _PubStatus(1);
}

void Function::_SubResult(shared_interfaces::msg::Float64Array::UniquePtr ptr) 
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "%s Error in _SubRequest(): initialize and subscribe result.",GetLocalTime());
        return;
    }

    RCLCPP_INFO(this->get_logger(),"%s subscribe a result: Float64Array", GetLocalTime());
    _PubResult(0, ptr->data);
    
}

void Function::_SubResultEx(shared_interfaces::msg::MeasurementResult::UniquePtr ptr) 
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "%s Error in _SubRequest(): initialize and subscribe result.",GetLocalTime());
        return;
    }

    RCLCPP_INFO(this->get_logger(),"%s subscribe a result: MeasurementResult", GetLocalTime());
    _PubResult(ptr->name, ptr->code, ptr->data);
}

void Function::_SubResultDefect(shared_interfaces::msg::Defect::UniquePtr ptr)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "%s Error in _SubRequest(): initialize and subscribe defect result.",GetLocalTime());
        return;
    }
    RCLCPP_INFO(this->get_logger(), "%s subscribe a result: value: %d, picture: %s.",GetLocalTime(), ptr->value, ptr->picture.c_str());
    _PubResult(0, ptr->value, ptr->picture);   
}

void Function::_SubResultEmbeddedParts(shared_interfaces::msg::EmbeddedParts::UniquePtr ptr) 
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "%s Error in _SubRequest(): initialize and subscribe embedded result.",GetLocalTime());
        return;
    }
    RCLCPP_INFO(this->get_logger(), "%s subscribe a result: value: %d, picture: %s.",GetLocalTime(), ptr->value, ptr->picture.c_str());
    _PubResult(0, ptr->value, ptr->picture); 
}

void Function::_PubResult(const int& code, const int& value, const std::string& imagePath)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "%s Error in _SubRequest(): initialize and publish a result.",GetLocalTime());
        return;
    }
    
    json j;
    j["code"] = code;
    j["context"]["id"] = _id;    
    j["context"]["result"] = json::object();

    json tmp; 
    if(_value == "DefectDetection" || _value == "EmbeddedParts")
    {
        tmp["name"] = _value;   
        tmp["value"] = value;
        tmp["picture"] = imagePath;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),"%s Error in _PubResult(): _value is invalid.",GetLocalTime());
        return;
    }

    j["context"]["result"] = tmp;

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = j.dump();
    _pubResult->publish(std::move(msg));
    RCLCPP_INFO(this->get_logger(), "%s publish result: value: %d, picture: %s.", GetLocalTime(), value, imagePath.c_str());
}

void Function::_PubResult(const int& code, const std::vector<double>& result)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "%s Error in _SubRequest(): initialize and publish a result.",GetLocalTime());
        return;
    }

    json j;
    j["code"] = code;
    j["context"]["id"] = _id;    
    j["context"]["result"] = json::object();

    json tmp; 

    tmp["name"] = _value;   
    tmp["code"] = 0;
    tmp["value"] = -1;
    tmp["a"] = -1.;
    tmp["b"] = -1.;
    tmp["c"] = -1.;
    tmp["d"] = -1.;


    if(code == 0)
    {            
        if(_value == "TopPlateFlatness" || _value == "GroundFlatness" || _value == "SideWallFlatness")
        {
            std::vector<double> tmp_result;
            tmp_result = result;
            std::sort(tmp_result.begin(), tmp_result.end());       
            auto sum = std::accumulate(tmp_result.begin(), tmp_result.end(), 0.0);
            std::vector<double> ratio(tmp_result.size());
            std::transform(tmp_result.begin(), tmp_result.end(), ratio.begin(), [sum](double x) {return x/sum;});
            std::vector<std::vector<double>> bin(5);
            for(std::size_t i = 0; i < ratio.size(); i++)
            {
                int index = (int)std::round(ratio[i] / 0.2);
                std::cout << "index: " << index << std::endl;
                bin[index].push_back(tmp_result[i]);
            }
            std::sort(bin.begin(), bin.end(), CompSize);

            std::cout << "bin[0] size: " << bin[0].size() << std::endl;
            double numberd = bin[0].size() / 2.0;
            double result_index;
            if(numberd == 0.5)
            {
                result_index = 0;
            }
            else if(numberd == 1.5)
            {
                result_index = 1;
            }
            else 
            {
                result_index = std::round(numberd);
            }

            
            tmp["value"] = bin[0][result_index];
            tmp["a"] = result[0];
            tmp["b"] = result[1];
            tmp["c"] = result[2];
            tmp["d"] = result[3];                
        }
        else if(_value == "StoreyHeight")
            tmp["value"] = result[0];
        else
            tmp["value"] = result[2];
      
        if(tmp["value"] == -1)
            j["code"] = 2;
    }

    j["context"]["result"] = tmp;

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = j.dump();
    _pubResult->publish(std::move(msg));
    
}

void Function::_PubResult(const std::string& name, const float& code, const std::vector<float>& result)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "%s Error in _SubRequest(): initialize and publish a result.",GetLocalTime());
        return;
    }

    std::cout << "name: " << name << std::endl;
    std::cout << "____ pub result" << std::endl;
    for(std::size_t i = 0; i < result.size(); i++)
    {
        std::cout << "result[" << i << "]: " << result[i] << std::endl;
    }

    json j;
    j["code"] = 0;
    j["context"]["id"] = _id;    
    j["context"]["result"] = json::object();

    json tmp; 

    tmp["name"] = name; 
    tmp["code"] = code; 
    tmp["value"] = -1.;
    tmp["a"] = -1.;
    tmp["b"] = -1.;
    tmp["c"] = -1.;
    tmp["d"] = -1.;
          
    if(name == "TopPlateFlatness" || name == "GroundFlatness" || name == "SideWallFlatness")
    {       
        tmp["value"] = result[0];
        tmp["a"] = result[0];
        tmp["b"] = result[1];
        tmp["c"] = result[2];
        tmp["d"] = result[3];                
    }
    else if(name == "WallVertical")
    {
        tmp["value"] = result[0];
        tmp["a"] = result[1];
        tmp["b"] = result[2];
        tmp["c"] = result[3];
        tmp["d"] = result[4]; 
    }
    else if(name == "DoorOpeningSize" || name == "WindowOpeningSize")
    {
        tmp["value"] = -1;
        tmp["a"] = result[0];
        tmp["b"] = result[1];
        tmp["c"] = -1;
        tmp["d"] = -1;
    }
    
    j["context"]["result"] = tmp;

    std::cout << "result: " << j.dump() << std::endl;
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = j.dump();
    _pubResult->publish(std::move(msg));
}

void Function::_PubStatus(const int& code, const std::string& status)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "%s Error in _SubRequest(): initialize and publish a status.",GetLocalTime());
        return;
    }

    json j;
    j["code"] = code;
    j["context"]["module"] = _value;
    j["context"]["status"] = "failure";
    j["context"]["statusNum"] = -1;

    if(code == 0)
        j["context"]["status"] = "idle";
    else if(code == 1)
        j["context"]["status"] = "busying";
    else
        j["context"]["status"] = "failure";

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = j.dump();
    _pubStatus->publish(std::move(msg));
    RCLCPP_INFO(this->get_logger(), "%s publish status: %s.", GetLocalTime(), j.dump().c_str());
}

}//namespace function

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(function::Function)

