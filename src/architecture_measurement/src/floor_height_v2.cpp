#include "architecture_measurement/floor_height.hpp"

#include "architecture_measurement/impl/file_io.hpp"
#include "architecture_measurement/impl/space_analytic_geometry.hpp"
#include "architecture_measurement/impl/cloud_pretreatment.hpp"
#include "architecture_measurement/impl/cloud_header.hpp"

#include "pcl/ModelCoefficients.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/transforms.h"

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

constexpr double THRESHOLD = std::sqrt(2)/2.0;

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

            ResetParameters();
            CalculateFixedParameters();
        }

        ~_Impl()
        {

        }
  
        void _InitializeParameters()
        {
            _node->declare_parameter<std::vector<double>>("I_R_C",{0.});
            _node->declare_parameter<std::vector<double>>("servoMotorRotationAxisVectorInCamera",{0.});
            _node->declare_parameter<std::vector<double>>("servoMotorOriginPointInCamera",{0.});

            return;
        }

        void _UpdateParameters()
        {
            std::vector<double> I_R_C_vector;
            std::vector<double> servoMotorRotationAxisVectorInCamera_vector;
            std::vector<double> servoMotorOriginPointInCamera_vector;
            
            _node->get_parameter("I_R_C",I_R_C_vector);
            _node->get_parameter("servoMotorRotationAxisVectorInCamera",servoMotorRotationAxisVectorInCamera_vector);
            _node->get_parameter("servoMotorOriginPointInCamera",servoMotorOriginPointInCamera_vector);

            double* I_R_C_vector_ptr = I_R_C_vector.data();
            I_R_C = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> >(I_R_C_vector_ptr);

            double* servoMotorRotationAxisVectorInCamera_vector_ptr = servoMotorRotationAxisVectorInCamera_vector.data();
            servoMotorRotationAxisVectorInCamera = Eigen::Map<Eigen::Matrix<double,3,1> >(servoMotorRotationAxisVectorInCamera_vector_ptr);

            double* servoMotorOriginPointInCamera_vector_ptr = servoMotorOriginPointInCamera_vector.data();
            servoMotorOriginPointInCamera = Eigen::Map<Eigen::Matrix<double,3,1> >(servoMotorOriginPointInCamera_vector_ptr);

            return;
        }

        void ResetParameters()
        {
            isFirst = true;

            roll_1 = 0;
            pitch_1 = 0;
            yaw_1 = 0;
            roll_2 = 0;
            pitch_2 = 0;
            yaw_2 = 0;

            height_1 = 0;
            height_2 = 0;
            height_3 = 0;

        }

        void CalculateFixedParameters()
        {
            if(servoMotorRotationAxisVectorInCamera(0) < 0)
                servoMotorRotationAxisVectorInCamera = -servoMotorRotationAxisVectorInCamera;
            
            Eigen::Vector3d C_AZ = servoMotorRotationAxisVectorInCamera;
            Eigen::Vector3d C_AX = Eigen::Vector3d::UnitZ().cross(C_AZ);
            Eigen::Vector3d C_AY = C_AZ.cross(C_AX);

            Eigen::Matrix3d C_R_A;
            C_R_A.col(0) = C_AX;
            C_R_A.col(1) = C_AY;
            C_R_A.col(2) = C_AZ;

            C_T_A.linear() = C_R_A;
            C_T_A.translation() = servoMotorOriginPointInCamera;

            Eigen::Vector3d translation{0.,0.,0.};
            I_T_C.linear() = I_R_C;
            I_T_C.translation() = translation;

            I_T_A = I_T_C * C_T_A;

        }

        void SortCloud(
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clouds,
            int& index, 
            Eigen::Vector3d& massCenter)
        {   
            //CloudHeader对象
            CloudHeaderVector cds(clouds.size());

            for(std::size_t i = 0; i < clouds.size(); i++)
            {        
                CloudHeader cd{clouds[i], i};
                cds[i] = cd;
            }
#ifdef TEST
            std::cout << "cds size (init): " << cds.size() << std::endl;
#endif //TEST

            //筛选水平面
            auto cds_end = std::remove_if(cds.begin(),cds.end(),
                [](CloudHeader cd)
                {
#ifdef TEST
                    std::cout << "cd.GetNormalVector(): " << cd.GetNormalVector().transpose() << std::endl;
                    std::cout << "Eigen::Vector3d::UnitZ(): " << Eigen::Vector3d::UnitZ().transpose() << std::endl;
                    std::cout << "dot value: " << std::fabs(cd.GetNormalVector().dot(Eigen::Vector3d::UnitZ())) << std::endl;
                    std::cout << "THRESHOLD: " << THRESHOLD << std::endl;
                    std::cout << "dot_value < THRESHOLD: " << bool(std::fabs(cd.GetNormalVector().dot(Eigen::Vector3d::UnitZ())) < THRESHOLD) << std::endl;
#endif //TEST

                    bool bool_value = std::fabs(cd.GetNormalVector().dot(Eigen::Vector3d::UnitZ())) < THRESHOLD;
                    return bool_value;
                });
            cds.erase(cds_end, cds.end());
            if(cds.empty())
            {
                std::string error = "Error in SortCloud():  cds size (remove horizontal plane) < 1.";
                std::string statusNumber = "-1";
                std::string status = GenerateStatusMessage(_node->get_name(), statusNumber, error);
                std::cout << "status: " << status << std::endl;
                _node->_PubStatus(status);

                throw std::runtime_error(error); 
            }

#ifdef TEST
            std::cout << "cds size (remove vertical plane): " << cds.size() << std::endl;

#endif //TEST

            //筛选最大点数量的平面
            std::sort(cds.begin(), cds.end(), SortSize);

            index = cds[0].GetIndex();
            massCenter = cds[0].GetMassCenter();

            std::cout << "finish sort cloud" << std::endl;

            return;
        }

        double CalculateVerticalDistance(double roll_radian, double pitch_radian, double yaw_radian)
        {
            // InitializeParameters(roll_radian, pitch_radian, yaw_radian);


            if(isFirst)
            {
                W_R_I_1 = GetMatrixFromRPY(roll_radian, pitch_radian, yaw_radian);
                W_T_I_1.linear() = W_R_I_1;
                W_T_I_1.translation() = translation;
                std::cout << "W_R_I_1: " <<  W_R_I_1 << std::endl;
                std::cout << "W_T_I_1: " <<  W_T_I_1.linear() << std::endl;
                std::cout << "W_T_I_1: " <<  W_T_I_1.translation() << std::endl;

                W_T_C_1 = W_T_I_1 * I_T_C;
                W_T_A_1 = W_T_C_1 * C_T_A;

                Eigen::Matrix3d I_R_A = I_T_A.linear();
                I_AZ = I_R_A.col(2);

                isFirst = false;
                return 0;
            }
            else
            {
                W_R_I_2 = GetMatrixFromRPY(roll_radian, pitch_radian, yaw_radian);
                W_T_I_2.linear() = W_R_I_2;
                W_T_I_2.translation() = translation;
                W_T_C_2 = W_T_I_2 * I_T_C;
                W_T_A_2 = W_T_C_2 * C_T_A;

                double Rx = I_AZ(0);
                double Ry = I_AZ(1);
                double Rz = I_AZ(2);

                double a1 = W_T_I_1.linear()(2,0);
                double a2 = W_T_I_1.linear()(2,1);
                double a3 = W_T_I_1.linear()(2,2);

                double a = -a1 * Rx * Rx - a2 * Rx * Ry + a3 * Rx * Rz + a3;
                double b = - a2 * Rz + a3 * Ry;
                double c = a1 * Rx * Rx + a2 * Rx * Ry + a3 * Rx * Rz - std::cos(pitch_2 + M_PI / 2.);

                std::cout << "Rx: " << Rx <<std::endl;
                std::cout << "Ry: " << Ry <<std::endl;
                std::cout << "Rz: " << Rz <<std::endl;
                std::cout << "a1: " << a1 <<std::endl;
                std::cout << "a2: " << a2 <<std::endl;
                std::cout << "a3: " << a3 <<std::endl;

                std::cout << "a: " << a <<std::endl;
                std::cout << "b: " << b <<std::endl;
                std::cout << "c: " << c <<std::endl;


                // double theta = Dichotomy(a,b,c);
                double theta = -150. / 180. * M_PI;
                std::cout << "theta: " << theta * 180 / M_PI << std::endl;

                Eigen::Matrix3d A1_R_A2;
                A1_R_A2 = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());

                Eigen::Transform<double,3,Eigen::Isometry> A1_T_A2;
                A1_T_A2.linear() = A1_R_A2;
                A1_T_A2.translation() << 0.,0.,0.;

                Eigen::Transform<double,3,Eigen::Isometry> W1_T_W2;
                W1_T_W2 = W_T_A_1 * A1_T_A2 * W_T_A_2.inverse();


                std::cout << "C_T_A: \n" << C_T_A.matrix() << std::endl;
                std::cout << "I_T_C: \n" << I_T_C.matrix() << std::endl;
                std::cout << "I_T_A: \n" << I_T_A.matrix() << std::endl;

                std::cout << "W_T_A_1: \n" << W_T_A_1.matrix() << std::endl;                
                std::cout << "W_T_A_2: \n" << W_T_A_2.matrix() << std::endl;                
                std::cout << "W1_T_W2: \n" << W1_T_W2.matrix() << std::endl;

                height_2 = W1_T_W2.translation()(2);

            }      

            std::cout << "height_1: " << height_1 << std::endl;
            std::cout << "height_2: " << -height_2 << std::endl;
            std::cout << "height_3: " << std::fabs(height_3) << std::endl;

            height = std::fabs(height_1) - height_2 + std::fabs(height_3);

            ResetParameters();

            return height;
        }

        std::vector<double> Calculate(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
        {
            if(cloud->points.empty())
            {
                std::string statusNumber = "-1";
                std::string error = "Error in Calculate(): cloud is empty.";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //倾角仪坐标系和世界水平坐标系之间的变换矩阵
            //添加一个锁
            double roll_radian = Radian(_node->_roll);
            double pitch_radian = Radian(_node->_pitch);
            double yaw_radian = Radian(_node->_yaw);

            Eigen::Matrix3d W_R_I = GetMatrixFromRPY(roll_radian, pitch_radian, yaw_radian);
            Eigen::Matrix3d W_R_C = W_R_I * I_R_C;
            Eigen::Matrix3f W_R_C_f = W_R_C.cast<float>();

            //点云处理对象
            CloudPretreatment cp;

            //体素滤波
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel_grid_filter(new pcl::PointCloud<pcl::PointXYZI>);
            cp.VoxelGridFilter(cloud,cloud_voxel_grid_filter);
            if(cloud_voxel_grid_filter -> points.empty())
            {
                std::string error = "Error in Calculate(): cloud_voxel_grid_filter is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //半径滤波
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_radius_filter(new pcl::PointCloud<pcl::PointXYZI>);
            cp.RadiusFilter(cloud_voxel_grid_filter,cloud_radius_filter);
            if(cloud_radius_filter -> points.empty())
            {
                std::string error = "Error in Calculate(): cloud_radius_filter is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //点云坐标变换
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transform(new pcl::PointCloud<pcl::PointXYZI>);
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.translation() << 0.0, 0.0, 0.0;
            transform.rotate (W_R_C_f);
            pcl::transformPointCloud (*cloud_radius_filter, *cloud_transform, transform);

            //法向量估计
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
            cp.NormalEstimation(cloud_transform,cloud_normal);

            //区域生长分割平面
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
            cp.PalneSegmentation(cloud_transform, cloud_normal, clouds);
            if(clouds.empty())
            {
                std::string error = "Error in Calculate(): clouds is empty()";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //筛选平面
            int index = -1;
            Eigen::Vector3d massCenter;
            SortCloud(clouds, index, massCenter);

            //extract
            if(index < 0)
            {
                std::string error = "Error in Calculate(): index < 0.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_optimal(clouds[index]);
            for(auto& p : *cloud_optimal)
            {
                p.intensity = 5;
            }

            if(isFirst)
            {
                height_1 = massCenter(2);
                std::cout << "isFirst: " << isFirst << std::endl;
                std::cout << "height_1: " << height_1 << std::endl;
                return {CalculateVerticalDistance(roll_radian, pitch_radian, yaw_radian)};
            }
            else
            {
                height_3 = massCenter(2);
                std::cout << "isFirst: " << isFirst << std::endl;
                std::cout << "height_3: " << height_3 << std::endl;
                return {CalculateVerticalDistance(roll_radian, pitch_radian, yaw_radian)};
            }
        }

        double Dichotomy(double a, double b, double c)
        {
            std::cout << "a: " << a << " b: " << b << " c: " << c <<std::endl;
            double lower_limit = -M_PI;
            double upper_limit = 0;

            double middle;
            double result = 100;

            while(std::fabs(result) > 0.001)
            {               

                middle = (lower_limit + upper_limit)/2.;

                double result1 = a * std::cos(lower_limit) + b * std::sin(lower_limit) + c;
                double result2 = a * std::cos(upper_limit) + b * std::sin(upper_limit) + c;

                result = a * std::cos(middle) + b * std::sin(middle) + c;

                std::cout << "\nresult: " << result << "result1: " << result1 << " result2: " << result2 << "\n" << std::endl; 
                if(result1 / result < 0)
                {
                    upper_limit = middle;
                }
                else if(result2 / result < 0)
                {
                    lower_limit = middle;
                }
                else
                {
                    throw std::runtime_error("error");
                }
            }

            return middle; 
        }

        void InitializeParameters(double _roll, double _pitch, double _yaw)
        {
            if(isFirst)
            {
                roll_1 = _roll;
                pitch_1 = _pitch;
                yaw_1 = _yaw;
            }
            else
            {
                roll_2 = _roll;
                pitch_2 = _pitch;
                yaw_2 = _yaw;
            }
        }



    private:
        Eigen::Matrix3d I_R_C;
        Eigen::Vector3d servoMotorRotationAxisVectorInCamera;
        Eigen::Vector3d servoMotorOriginPointInCamera;
        
        Eigen::Transform<double,3,Eigen::Isometry> C_T_A;
        Eigen::Transform<double,3,Eigen::Isometry> I_T_C;
        Eigen::Transform<double,3,Eigen::Isometry> I_T_A;

        Eigen::Matrix3d W_R_I_1, W_R_I_2;
        Eigen::Transform<double,3,Eigen::Isometry> W_T_I_1, W_T_I_2;
        Eigen::Transform<double,3,Eigen::Isometry> W_T_C_1, W_T_C_2;
        Eigen::Transform<double,3,Eigen::Isometry> W_T_A_1, W_T_A_2;
        Eigen::Vector3d translation{0.,0.,0.};

        Eigen::Vector3d I_AZ;

        bool isFirst;
        double roll_1, pitch_1, yaw_1, roll_2, pitch_2, yaw_2;
        double height_1, height_2, height_3, height;

        FloorHeight* _node;
};

/**** floorHeight ****/

FloorHeight::FloorHeight(const rclcpp::NodeOptions& option = rclcpp::NodeOptions()) : Node("floor_height_node", option)
{
    _initThread = std::thread(&FloorHeight::_Init, this);
}

FloorHeight::~FloorHeight()
{
    try
    {
        _initThread.join();

        _subRpy.reset();
        // _subLaserRanger.reset();
        _subPointCloud.reset();
        _pubResult.reset();
        _pubStatus.reset();

        RCLCPP_INFO(this->get_logger(),"FloorHeight destroyed successfully");
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Exception in FloorHeight destructor: %s", e.what());
    }
    catch(...)
    {
        RCLCPP_ERROR(this->get_logger(),"Exception in FloorHeight destructor: unknown");
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

    // _srvMeasure = this->create_service<shared_interfaces::srv::TriggerOp>(_srvMeasureName, std::bind(&FloorHeight::_SrvMeasure,this,std::placeholders::_1, std::placeholders::_2));

    _subRpy = this->create_subscription<shared_interfaces::msg::RollPitchYaw>(_subRpyName, 10, std::bind(&FloorHeight::_SubRpy, this, std::placeholders::_1));
    // _subLaserRanger = this->create_subscription<shared_interfaces::msg::LaserRanger>(_subLaserRangerName, 10, std::bind(&FloorHeight::_SubLaserRanger, this, std::placeholders::_1));
    _subPointCloud = this->create_subscription<shared_interfaces::msg::PointCloudOp>(_subPointCloudName, 1, std::bind(&FloorHeight::_SubPointCloud, this, std::placeholders::_1));

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

// void FloorHeight::_SrvMeasure(const std::shared_ptr<shared_interfaces::srv::TriggerOp::Request>, std::shared_ptr<shared_interfaces::srv::TriggerOp::Response> response)
// {
//     if(_status < 0)
//     {
//         RCLCPP_INFO(this->get_logger(),"%s Error in destruction: initialize and receive a service call.",GetLocalTime());
//         return;
//     }

//     response->success = false;
//     response->message = "Fail: measure part2";

//     _PubResult(_impl->Calculate());

//     response->success = true;
//     response->message = "Success: measure part2";
// }

void FloorHeight::_SubRpy(shared_interfaces::msg::RollPitchYaw::UniquePtr ptr)
{
    //添加一个锁
    _roll = ptr->roll;
    _pitch = ptr->pitch;
    _yaw = ptr->yaw;

    return;
}

// void FloorHeight::_SubLaserRanger(shared_interfaces::msg::LaserRanger::UniquePtr ptr)
// {
//     //添加一个锁
//     _laserRanger = ptr->distance;    
//     _PubResult(_impl->Calculate());
//     RCLCPP_INFO(this->get_logger(), "%s subscribe a laser ranger: %f", GetLocalTime(), ptr->distance);
//     return;
// }

void FloorHeight::_SubPointCloud(shared_interfaces::msg::PointCloudOp::UniquePtr ptr)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"initialize and sub a subscribe cloud.");
        return;
    }
    const std::string& option = (ptr->option).data.c_str();

    std::cout << "option: " << option << std::endl;

    if( option == "StoreyHeight")
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        unsigned int width = ptr->width;
        unsigned int height = ptr->height;

        unsigned int point_cloud_number = width * height;

        cloud->points.resize(point_cloud_number);

        for(std::size_t i = 0; i < point_cloud_number; i++)
        {
            pcl::PointXYZI point;
            point.x = (ptr->x)[i];
            point.y = (ptr->y)[i];
            point.z = (ptr->z)[i];
            point.intensity = (ptr->intensity)[i];
            cloud->points[i] = point;
        }

        _PubResult(_impl->Calculate(cloud));  
    }

    return;
}

void FloorHeight::_PubResult(const std::vector<double>& result)
{
    auto msg = std::make_unique<shared_interfaces::msg::Float64Array>();
    msg->data = result;
    _pubResult->publish(std::move(msg));
    RCLCPP_INFO(this->get_logger(), "%s publish a result: %f", GetLocalTime(), result[0]);
    return;
}

void FloorHeight::_PubStatus(const std::string& status)
{
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