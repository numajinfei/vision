#include "architecture_measurement/floor_height.hpp"

#include "architecture_measurement/impl/file_io.hpp"
#include "architecture_measurement/impl/space_analytic_geometry.hpp"
#include "architecture_measurement/impl/cloud_pretreatment.hpp"
#include "architecture_measurement/impl/cloud_header.hpp"

#include <chrono>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <exception>
#include <deque>
#include <thread>
#include <mutex>

#include "Eigen/Dense"
#include "nlohmann/json.hpp"

namespace am
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

std::string GenerateStatusMessage(const std::string name, std::string& statusNum, std::string& message)
{
    json j;
    j["context"]["module"] = name;
    j["context"]["status"] = "error";
    j["context"]["statusNum"] = statusNum;
    j["context"]["message"] = message;

    return j.dump();
}

class FloorHeight::_Impl
{
    public:
        explicit _Impl(FloorHeight * ptr) : _node(ptr)
        {
            _InitializeParameters();
            _UpdateParameters();
        }

        ~_Impl()
        {

        }
  
        void _InitializeParameters()
        {
            _node->declare_parameter<std::vector<double>>("I_R_C",{});
            _node->declare_parameter<std::vector<double>>("laserLineVectorInCamera",{});
            _node->declare_parameter<std::vector<double>>("laserLineOriginPointInCamera",{});
            _node->declare_parameter<std::vector<double>>("servoMotorRotationAxisVectorInCamera",{});
            _node->declare_parameter<std::vector<double>>("servoMotorOriginPointInCamera",{});
            _node->declare_parameter<double>("machineHeight",0.);
            _node->declare_parameter<std::string>("fileName", "");

            return;
        }

        void _UpdateParameters()
        {
            std::vector<double> I_R_C_vector;
            std::vector<double> laserLineVectorInCamera_vector;
            std::vector<double> laserLineOriginPointInCamera_vector;
            std::vector<double> servoMotorRotationAxisVectorInCamera_vector;
            std::vector<double> servoMotorOriginPointInCamera_vector;
            
            _node->get_parameter("I_R_C",I_R_C_vector);
            _node->get_parameter("laserLineVectorInCamera",laserLineVectorInCamera_vector);
            _node->get_parameter("laserLineOriginPointInCamera",laserLineOriginPointInCamera_vector);
            _node->get_parameter("servoMotorRotationAxisVectorInCamera",servoMotorRotationAxisVectorInCamera_vector);
            _node->get_parameter("servoMotorOriginPointInCamera",servoMotorOriginPointInCamera_vector);
            _node->get_parameter("machineHeight", machineHeight);
            _node->get_parameter("fileName",fileName);

            double* I_R_C_vector_ptr = I_R_C_vector.data();
            I_R_C = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> >(I_R_C_vector_ptr);

            double* laserLineVectorInCamera_vector_ptr = laserLineVectorInCamera_vector.data();
            laserLineVectorInCamera = Eigen::Map<Eigen::Matrix<double,3,1> >(laserLineVectorInCamera_vector_ptr);

            double* laserLineOriginPointInCamera_vector_ptr = laserLineOriginPointInCamera_vector.data();
            laserLineOriginPointInCamera = Eigen::Map<Eigen::Matrix<double,3,1> >(laserLineOriginPointInCamera_vector_ptr);

            double* servoMotorRotationAxisVectorInCamera_vector_ptr = servoMotorRotationAxisVectorInCamera_vector.data();
            servoMotorRotationAxisVectorInCamera = Eigen::Map<Eigen::Matrix<double,3,1> >(servoMotorRotationAxisVectorInCamera_vector_ptr);

            double* servoMotorOriginPointInCamera_vector_ptr = servoMotorOriginPointInCamera_vector.data();
            servoMotorOriginPointInCamera = Eigen::Map<Eigen::Matrix<double,3,1> >(servoMotorOriginPointInCamera_vector_ptr);

            return;
        }

        double CalculateVerticalDistance()
        {
            double laserRanger = _node->_laserRanger * 1000;
            double rollRadian = Radian(_node->_roll);
            double pitchRadian = Radian(_node->_pitch);
            double yawRadian = Radian(_node->_yaw);

            Eigen::Matrix3d W_R_I = GetMatrixFromRPY(rollRadian, pitchRadian, yawRadian);
            Eigen::Matrix3d W_R_C = W_R_I * I_R_C;

            if(laserLineVectorInCamera(2) < 0)
                laserLineVectorInCamera = - laserLineVectorInCamera;

            Eigen::Vector3d laserLineVectorInWorld = W_R_C * laserLineVectorInCamera;
            Eigen::Vector3d laserLineOriginPointInWorld = W_R_C * laserLineOriginPointInCamera;
            Eigen::Vector3d servoMotorOriginPointInWorld = W_R_C * servoMotorOriginPointInCamera;

            Eigen::Vector3d unitZ = Eigen::Vector3d::UnitZ();
            double distance = laserRanger * (laserLineVectorInWorld.dot(unitZ)/unitZ.norm()/laserLineVectorInWorld.norm());

            double deltaZ = laserLineOriginPointInWorld(2) - servoMotorOriginPointInWorld(2);
            double height = distance + deltaZ + machineHeight;
            std::cout << "distance: " << distance << std::endl;
            std::cout << "deltaZ: " << deltaZ << std::endl;
            std::cout << "machineHeight: " << machineHeight << std::endl;

            return height;
        }

        std::vector<double> Calculate()
        {
            return {CalculateVerticalDistance()};
        }

    private:
        Eigen::Matrix3d I_R_C;
        Eigen::Vector3d laserLineVectorInCamera;
        Eigen::Vector3d laserLineOriginPointInCamera;
        Eigen::Vector3d servoMotorRotationAxisVectorInCamera;
        Eigen::Vector3d servoMotorOriginPointInCamera;
        double machineHeight;

        std::string fileName;

        FloorHeight* _node;
};

/**** floorHeight ****/

FloorHeight::FloorHeight(const rclcpp::NodeOptions& option) : Node("floor_height_node", option)
{
    _initThread = std::thread(&FloorHeight::_Init, this);
}

FloorHeight::~FloorHeight()
{
    try
    {
        _initThread.join();

        _subRpy.reset();
        _subLaserRanger.reset();
        _pubResult.reset();
        _pubStatus.reset();

        RCLCPP_INFO(this->get_logger(),"%s FloorHeight destroyed successfully", GetLocalTime());
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(),"%s Exception in FloorHeight destructor: %s", GetLocalTime(), e.what());
        rclcpp::shutdown();
    }
    catch(...)
    {
        RCLCPP_INFO(this->get_logger(),"%s Exception in FloorHeight destructor: unknown", GetLocalTime());
        rclcpp::shutdown();
    }    
}

void FloorHeight::_Init() try
{
    _status = -1;

    _InitializeParameters();
    _UpdateParameters();
    
    _laserRanger = 0;
    _roll = 0;
    _pitch = 0;
    _yaw = 0;    

    _subRpy = this->create_subscription<shared_interfaces::msg::RollPitchYaw>(_subRpyName, 5, std::bind(&FloorHeight::_SubRpy, this, std::placeholders::_1));
    _subLaserRanger = this->create_subscription<shared_interfaces::msg::LaserRanger>(_subLaserRangerName, 1, std::bind(&FloorHeight::_SubLaserRanger, this, std::placeholders::_1));
    _pubResult = this->create_publisher<shared_interfaces::msg::Float64Array>(_pubResultName, 1);
    _pubStatus = this->create_publisher<std_msgs::msg::String>(_pubStatusName, 1);

    _impl = std::make_unique<_Impl>(this);

    _status = 0;

    RCLCPP_INFO(this->get_logger(), "FloorHeight initialized successfully");  
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(this->get_logger(), "%s Exception in FloorHeight initializer: %s", GetLocalTime(), e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(this->get_logger(), "%s Exception in FloorHeight initializer: unknown.",GetLocalTime());
    rclcpp::shutdown();
}  

void FloorHeight::_InitializeParameters()
{
    return;
}

void FloorHeight::_UpdateParameters()
{
    return;
}

void FloorHeight::_SubRpy(shared_interfaces::msg::RollPitchYaw::UniquePtr ptr)
{
    if(_status < 0)
        return;
    
    _roll = ptr->roll;
    _pitch = ptr->pitch;
    _yaw = ptr->yaw;
    return;
}

void FloorHeight::_SubLaserRanger(shared_interfaces::msg::LaserRanger::UniquePtr ptr)
{
    if(_status < 0)
        return;
    
    _laserRanger = ptr->distance;    
    _PubResult(_impl->Calculate());
    RCLCPP_INFO(this->get_logger(), "%s subscribe a laser ranger: %f", GetLocalTime(), ptr->distance);
    return;
}

void FloorHeight::_PubResult(const std::vector<double>& result)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "%s initialize and sub a laserRanger.", GetLocalTime());
        return;
    }

    auto msg = std::make_unique<shared_interfaces::msg::Float64Array>();
    msg->data = result;
    _pubResult->publish(std::move(msg));
    RCLCPP_INFO(this->get_logger(), "%s publish a result: %f", GetLocalTime(), result[0]);
    return;
}

void FloorHeight::_PubStatus(const std::string& status)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "%s initialize and pub a status.", GetLocalTime());
        return;
    }

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = status;
    _pubStatus->publish(std::move(msg));
    RCLCPP_INFO(this->get_logger(), "%s publish a status: %s", GetLocalTime(), status.c_str());
    return;
}


}//namespace am

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(am::FloorHeight)