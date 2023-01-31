#include "architecture_measurement/levelness.hpp"

#include "architecture_measurement/impl/file_io.hpp"
#include "architecture_measurement/impl/space_analytic_geometry.hpp"
#include "architecture_measurement/impl/cloud_pretreatment.hpp"
#include "architecture_measurement/impl/cloud_header.hpp"

#include "pcl/ModelCoefficients.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/project_inliers.h"

#include "nlohmann/json.hpp"

#include <chrono>
#include <fstream>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;
#define TEST

namespace am
{

using namespace std::chrono_literals;
using json = nlohmann::json;


constexpr int POINT_SIZE = 100;

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

bool SortPointCloudYL(pcl::PointXYZI& point1, pcl::PointXYZI& point2)
{
    return point1.y < point2.y;
}

bool SortPointCloudYG(pcl::PointXYZI& point1, pcl::PointXYZI& point2)
{
    return point1.y > point2.y;
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


class Levelness::_Impl
{
    public:
        explicit _Impl(Levelness * ptr) : _node(ptr)
        {
            _InitializeParameters();
            _UpdateParameters();

            status = 0;
        }

        ~_Impl()
        {

        }

        void _InitializeParameters()
        {
            _node->declare_parameter<std::vector<double>>("I_R_C",{0.});
            _node->declare_parameter<std::string>("fileName","file");
            _node->declare_parameter<double>("extraParam_roof",0.0);
            _node->declare_parameter<double>("extraParam_ground",0.0);
            _node->declare_parameter<std::vector<std::string>>("engineering", {"Normal"});
            _node->declare_parameter<std::vector<std::string>>("option",{"StoreyHeight"});
        }

        void _UpdateParameters()
        {
            std::vector<double> I_R_C_vector;
            
            _node->get_parameter("I_R_C",I_R_C_vector);

            double* I_R_C_vector_ptr = I_R_C_vector.data();
            I_R_C = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> >(I_R_C_vector_ptr);

            _node->get_parameter("fileName",fileName);

            _node->get_parameter("extraParam_roof", extraParam_roof);
            _node->get_parameter("extraParam_ground", extraParam_ground);

            _node->get_parameter("engineering", _node->_engineeringVec);
            _node->get_parameter("option", _node->_optionVec); 
        }

        void SortCloud(
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clouds,
            const int& type, 
            int& index, 
            Eigen::Vector4d& planeCoeff)
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

            //筛选顶板或地面
            switch (type)
            {
            case GROUND:
#ifdef TEST
                std::cout << "type: GROUND." << std::endl; 
#endif //TEST
                cds_end = std::remove_if(cds.begin(),cds.end(),
                    [](CloudHeader cd)
                    {
#ifdef TEST
                        std::cout << "cd.GetMassCenter()(2): " << cd.GetMassCenter()(2) << std::endl;
                        std::cout << "cd.GetMassCenter()(2) > 0: " << bool(cd.GetMassCenter()(2) > 0) << std::endl;
#endif //TEST
                        return cd.GetMassCenter()(2) > 0;
                    });//去除顶板平面
                break;
            case ROOF:
#ifdef TEST
                std::cout << "type: ROOF." << std::endl; 
#endif //TEST
                cds_end = std::remove_if(cds.begin(),cds.end(),
                    [](CloudHeader cd)
                    {
#ifdef TEST
                        std::cout << "cd.GetMassCenter()(2): " << cd.GetMassCenter()(2) << std::endl;
                        std::cout << "cd.GetMassCenter()(2) < 0: " << bool(cd.GetMassCenter()(2) < 0) << std::endl;
#endif //TEST
                        return cd.GetMassCenter()(2) < 0;
                    });//去除地面平面
                break;            
            default:
                std::string error = "Error in SortCloud(): type is invalid.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
                break;
            }
            cds.erase(cds_end, cds.end());
            if(cds.empty())
            {
                std::string error = "Error in SortCloud():  cds size (remove floor or roof) < 1.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }
#ifdef TEST
            else
            {
                std::cout << "cds size (remove floor or roof): " << cds.size() << std::endl;

                FileIO ofile;
                for(std::size_t i = 0; i < cds.size(); i++)
                {
                    std::string name = "/home/ubuntu/tmp/levelness/levelness" + std::to_string(i) + ".txt";
                    ofile.SetProperty(name);
                    ofile.Write(cds[i].GetPointCloudPtr());
                } 
            }
           
#endif //TEST
            std::cout << "cds size: " << cds.size() << std::endl;
            std::this_thread::sleep_for(100ms);
            //筛选最大点数量的平面
            std::sort(cds.begin(), cds.end(), SortSize);

            index = cds[0].GetIndex();
            planeCoeff = cds[0].GetPlaneCoeff();

            std::cout << "finish sort cloud" << std::endl;

            return;
        }


        std::vector<double> Calculate(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,const int& type) try
        {
            if(cloud->points.empty())
            {
                std::string statusNumber = "-1";
                std::string error = "Error in Calculate(): cloud is empty.";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //倾角仪坐标系和世界水平坐标系之间的变换矩阵
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
            Eigen::Vector4d planeCoeff;
            SortCloud(clouds, type, index, planeCoeff);

            //extract
            if(index < 0)
            {
                std::string error = "Error in Calculate(): index < 0.";
                throw std::runtime_error(error);
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_optimal(clouds[index]);
            for(auto& p : *cloud_optimal)
            {
                p.intensity = 5;
            }

            Plane3d plane3d{planeCoeff[0], planeCoeff[1], planeCoeff[2], planeCoeff[3]};//平面参数
            Eigen::Vector3d camera_axis_y{0.,1.,0.};//相机坐标系Y轴矢量
            Eigen::Vector3d camera_axis_y_in_world = W_R_C * camera_axis_y;//相机坐标系Y轴矢量在水平坐标系下的矢量
            Plane3d camera_plane_xoz_in_world{camera_axis_y_in_world(0),camera_axis_y_in_world(1),camera_axis_y_in_world(2),0.};//构造平面对象
            DoublePlanes doublePlanes{plane3d, camera_plane_xoz_in_world};//构造双平面对象
            Line3d intersection_line;//交线对象
            doublePlanes.GetIntersectionLine(intersection_line);//计算交线
            std::cout << "intersection_line: " << intersection_line.vector.vector.transpose() << std::endl;
            std::cout << "intersection_line: " << intersection_line.point.point.transpose() << std::endl;

            Eigen::Vector3d fittingPlaneX_eigen = intersection_line.vector.vector;//在拟合平面上构造X轴
            Eigen::Vector3d fittingPlaneZ_eigen;
            fittingPlaneZ_eigen << planeCoeff[0],planeCoeff[1],planeCoeff[2];//拟合平面法矢构造Z轴
            Eigen::Vector3d fittingPlaneY_eigen = fittingPlaneZ_eigen.cross(fittingPlaneX_eigen);//构造拟合平面Y轴
            std::cout << "fittingPlaneX_eigen: " << fittingPlaneX_eigen.transpose() << std::endl;
            std::cout << "fittingPlaneZ_eigen: " << fittingPlaneZ_eigen.transpose() << std::endl;
            std::cout << "fittingPlaneY_eigen: " << fittingPlaneY_eigen.transpose() << std::endl;

            double d = - fittingPlaneY_eigen.dot(intersection_line.point.point);//在构造的拟合平面坐标系上计算XOZ平面在水平坐标系下的参数

            std::vector<double> result;//水平度结果
            pcl::PointCloud<pcl::PointXYZI>::Ptr rulers(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云

            for(int i = -2; i < 3; i++)
            {
                double limit_l = d + i * 2 * 50 - 25;//靠尺下边界
                double limit_s = d + i * 2 * 50 + 25;//靠尺上边界
                std::cout << "limit_l: " << limit_l << std::endl;
                std::cout << "limit_s: " << limit_s << std::endl;
                Eigen::Vector3f fittingPlaneY_eigen_f = fittingPlaneY_eigen.cast<float>();
                Eigen::Vector4f plane_s{fittingPlaneY_eigen_f(0), fittingPlaneY_eigen_f(1), fittingPlaneY_eigen_f(2), (float)limit_s};//靠尺上边界平面
                Eigen::Vector4f plane_l{fittingPlaneY_eigen_f(0), fittingPlaneY_eigen_f(1), fittingPlaneY_eigen_f(2), (float)limit_l};//靠尺下边界平面
                
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);
                cp.PlaneClipper3D(cloud_optimal, cloud_tmp, plane_l,true);//提取靠尺点云
                if(cloud_tmp->size() < POINT_SIZE)
                {
                    std::cout << "cloud_tmp size: " << cloud_tmp->size() << std::endl;
                    result.push_back(-1);
                    continue;
                    // std::string error = "Error in Calculate(): cloud_tmp size < POINT_SIZE.";
                    // throw std::runtime_error(error);
                }       
                pcl::PointCloud<pcl::PointXYZI>::Ptr ruler(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云         
                cp.PlaneClipper3D(cloud_tmp, ruler, plane_s, false);//提取靠尺点云
                if(ruler->size() < POINT_SIZE)
                {
                   std::cout << "ruler size: " << ruler-> size() << std::endl;
                    result.push_back(-1);
                    continue;
                    // std::string error = "Error in Calculate(): ruler size < POINT_SIZE.";
                    // throw std::runtime_error(error);
                }   

                //计算靠尺长度
                pcl::ModelCoefficients::Ptr ruler_plane_coef(new pcl::ModelCoefficients);
                ruler_plane_coef->values.resize(4);
                ruler_plane_coef->values[0] = plane_s(0);
                ruler_plane_coef->values[1] = plane_s(1);
                ruler_plane_coef->values[2] = plane_s(2);
                ruler_plane_coef->values[3] = plane_s(3);

                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ruler_projected(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::ProjectInliers<pcl::PointXYZI> proj;
                proj.setModelType(pcl::SACMODEL_PLANE);
                proj.setInputCloud(ruler);
                proj.setModelCoefficients(ruler_plane_coef);
                proj.filter(*cloud_ruler_projected);

                std::sort(cloud_ruler_projected->points.begin(), cloud_ruler_projected->points.end(), SortPointCloudYL);
                auto point_beign = *cloud_ruler_projected->points.begin();
                auto point_end = *cloud_ruler_projected->points.rbegin();
                Eigen::Vector3d point_begin_vector, point_rbegin_vector;
                point_begin_vector << point_beign.x, point_beign.y ,point_beign.z;
                point_rbegin_vector << point_end.x, point_end.y, point_end.z;
                double ruler_distance = (point_begin_vector-point_rbegin_vector).norm();                

                *rulers += *ruler;

                pcl::ModelCoefficients ruler_coeff = cp.Plane(ruler);//靠尺点云拟合平面
                Eigen::Vector3d ruler_normal_vector_in_world{ruler_coeff.values[0], ruler_coeff.values[1], ruler_coeff.values[2]};//靠尺点云平面法向量

//todo
                if(ruler_normal_vector_in_world(2) < 0)
                    ruler_normal_vector_in_world = -1 * ruler_normal_vector_in_world;                
                if(fittingPlaneX_eigen(1) < 0)
                    fittingPlaneX_eigen = -1 * fittingPlaneX_eigen;
                Eigen::Vector3d eigenVectorMiddle = fittingPlaneX_eigen.cross(ruler_normal_vector_in_world);
                std::cout << "std::acos(eigenVectorMiddle.dot(Eigen::Vector3d::UnitZ())): " << std::acos(eigenVectorMiddle.dot(Eigen::Vector3d::UnitZ())) << std::endl;
                double rotateAngle = 0.;
                rotateAngle = M_PI / 2. - std::acos(eigenVectorMiddle.dot(Eigen::Vector3d::UnitZ()));
                Eigen::Matrix3d rotateMatrix;
                rotateMatrix = Eigen::AngleAxisd(rotateAngle, fittingPlaneX_eigen);
                Eigen::Vector3d normalVectorOptimal = rotateMatrix * ruler_normal_vector_in_world;
                double angle_radian = std::acos(std::fabs(Eigen::Vector3d::UnitZ().dot(normalVectorOptimal)));//计算靠尺点云法矢与水平坐标系Z轴之间的夹角
#ifdef TEST
                std::cout << "ruler_normal_vector_in_world: " << ruler_normal_vector_in_world.transpose() << std::endl;
                std::cout << "fittingPlaneX_eigen: " << fittingPlaneX_eigen.transpose() << std::endl;
                std::cout << "eigenVectorMiddle: " << eigenVectorMiddle.transpose() << std::endl;
                std::cout << "rotateAngle: " << rotateAngle * 180 / M_PI << std::endl;
                std::cout << "normalVectorOptimal: " << normalVectorOptimal.transpose() << std::endl;

#endif //TEST
//todo

                //double angle_radian = std::acos(std::fabs(Eigen::Vector3d::UnitZ().dot(ruler_normal_vector_in_world)));//计算靠尺点云法矢与水平坐标系Z轴之间的夹角
                // if(type == GROUND)
                //     angle_radian -= extraParam_ground/180.*M_PI; 
                // else if(type == ROOF)
                //     angle_radian -= extraParam_roof/180.*M_PI;
                
                // angle_radian = std::fabs(angle_radian);

                double angle_degree = Degree(angle_radian);
                std::cout << "angle_degree: " << angle_degree <<std::endl;
                angle_degree = std::fabs(angle_degree);//将夹角转换为角度，并求与90度之间的偏差
                // std::cout << "angle_degree: " << angle_degree << std::endl;

                double one_result = ruler_distance * std::tan(std::fabs(angle_radian)) ;
                std::cout << "ruler_distance: " << ruler_distance << std::endl;
                std::cout << "one_result: " << one_result << std::endl;

                result.push_back(one_result);
            }

            //测试部分
            std::string time = GetLocalTime();
            std::string point_cloud_origin_name = "/home/ubuntu/tmp/levelness/pointcloud_origin_" + time + ".txt";
            std::string point_cloud_transform_name = "/home/ubuntu/tmp/levelness/pointcloud_transform_" + time + ".txt";
            std::string point_cloud_optimal_name = "/home/ubuntu/tmp/levelness/pointcloud_optimal_" + time + ".txt";
            std::string point_cloud_ruler_name = "/home/ubuntu/tmp/levelness/pointcloud_ruler_" + time + ".txt";

            FileIO fo;

            for(auto& p : *cloud_optimal)
            {
                p.intensity = 5;
            }

            for(auto& p : *rulers)
            {
                p.intensity = 0;
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr file_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            *file_cloud += *cloud_optimal;
            *file_cloud += *rulers;

            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_transform(new pcl::PointCloud<pcl::PointXYZI>);
            cp.PointcloudTransform(*file_cloud, *ground_cloud_transform, 0);

            //保存点云数据用于显示
            fo.SetProperty(fileName);
            std::cout << "fileName: " << fileName << std::endl;
            fo.Write(ground_cloud_transform);

#ifdef TEST
            fo.SetProperty(point_cloud_origin_name);
            fo.Write(cloud_voxel_grid_filter);

            fo.SetProperty(point_cloud_transform_name);
            fo.Write(cloud_transform);          

            fo.SetProperty(point_cloud_optimal_name);
            fo.Write(cloud_optimal);

            fo.SetProperty(point_cloud_ruler_name);
            fo.Write(rulers);

            for(std::size_t i = 0; i < result.size(); i++)
            {
                std::cout << "result[" << i << "]: " << result[i] << std::endl;
            }
#endif //TEST


            return {result};
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO(_node->get_logger(),"%s %s.",GetLocalTime(),e.what());
            return {-1, -1, -1, -1, -1};
        }
        catch(...)
        {
            RCLCPP_INFO(_node->get_logger(),"%s unknown error.",GetLocalTime());
            return {-1, -1, -1, -1, -1};
        }

    private:
        int status;

        float extraParam_roof;
        float extraParam_ground;

        Eigen::Matrix3d I_R_C;

        std::string fileName;

        Levelness* _node;
};

/**** Levelness ****/
Levelness::Levelness(const rclcpp::NodeOptions& option  = rclcpp::NodeOptions()) : rclcpp::Node("perpendicularity_and_levelness_node", option)
{
    _initThread = std::thread(&Levelness::_Init, this);
}

Levelness::~Levelness() try
{
    _initThread.join();

    _subPointCloud.reset();
    _subRpy.reset();
    _pubResult.reset();
    _pubStatus.reset();

    RCLCPP_INFO(this->get_logger(),"%s Levelness destroyed successfully.", GetLocalTime());
}
catch(const std::exception& e)
{
    RCLCPP_INFO(this->get_logger(), "%s Exception in Levelness destruction: %s", GetLocalTime(), e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_INFO(this->get_logger(),"%s Exception in Levelness destruction: unknown", GetLocalTime());
    rclcpp::shutdown();
}

void Levelness::_Init() try
{
    _InitializeParameters();
    _UpdateParameters();

    _status = -1;

    _roll = 0;
    _pitch = 0;
    _yaw = 0;

    _impl = std::make_unique<_Impl>(this);

    _subPointCloud = this->create_subscription<shared_interfaces::msg::PointCloudC>(
        _subPointCloudName, 
        1, 
        std::bind(&Levelness::_SubPointCloud, this, std::placeholders::_1));

    _subRpy = this->create_subscription<shared_interfaces::msg::RollPitchYaw>(
        _subRpyName, 
        10, 
        std::bind(&Levelness::_SubRpy, this, std::placeholders::_1)
    );

    _pubResult = this->create_publisher<shared_interfaces::msg::Float64Array>(_pubResultName, 1);

    _pubStatus = this->create_publisher<std_msgs::msg::String>(_pubStatusName, 1);

    _status = 0;

    RCLCPP_INFO(this->get_logger(), "%s Levelness initialized successfully.", GetLocalTime());
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(this->get_logger(), "%s Exception in Levelness initializer: %s", GetLocalTime(), e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(this->get_logger(),"%s Exception in Levelness initializer: unknown", GetLocalTime());
    rclcpp::shutdown();
}

void Levelness::_InitializeParameters()
{
    return;
}

void Levelness::_UpdateParameters()
{
    return;
}

void Levelness::_SubPointCloud(shared_interfaces::msg::PointCloudC::UniquePtr pointCloudCPtr)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"initialize and sub a subscribe cloud.");
        return;
    }

    const long id = pointCloudCPtr->id;
    const std::string engineering = pointCloudCPtr->engineering.c_str();
    const std::string option = pointCloudCPtr->option.c_str();
    const float angle = pointCloudCPtr->angle;
    const int frameID = pointCloudCPtr->frame_id;

    {
        auto iter = std::find_if(_engineeringVec.begin(), _engineeringVec.end(), [&engineering](const std::string& engineeringVec)
        {
            return EndsWith(engineeringVec, engineering);
        });
        if(iter == _engineeringVec.end())
            return;
    }
    {
        auto iter = std::find_if(_optionVec.begin(), _optionVec.end(), [&option](const std::string& optionVec)
        {
            return EndsWith(optionVec, option);
        });
        if(iter == _optionVec.end())
            return;
    }
    std::unordered_map<std::string, int> umapEng = LUTEngineering();
    std::unordered_map<std::string, int>::const_iterator iterEng = umapEng.find(engineering);
    if(iterEng == umapEng.end())
        return;
    const int ENGINEER = iterEng->second;

    std::unordered_map<std::string, int> umapOp = LUTOption();
    std::unordered_map<std::string, int>::const_iterator iterOp = umapOp.find(option);
    if(iterOp == umapOp.end())
        return;
    const int OBJECT = iterOp->second;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    unsigned int width = pointCloudCPtr->width;
    unsigned int height = pointCloudCPtr->height;

    unsigned int point_cloud_number = width * height;

    cloud->points.resize(point_cloud_number);

    for(std::size_t i = 0; i < point_cloud_number; i++)
    {
        pcl::PointXYZI point;
        point.x = (pointCloudCPtr->x)[i];
        point.y = (pointCloudCPtr->y)[i];
        point.z = (pointCloudCPtr->z)[i];
        point.intensity = (pointCloudCPtr->intensity)[i];
        cloud->points[i] = point;
    }

    _PubResult(_impl->Calculate(cloud, OBJECT));

    return;  
}

void Levelness::_SubRpy(shared_interfaces::msg::RollPitchYaw::UniquePtr ptr)
{
    if(_status < 0)
    {
        return;
    }
    _roll = ptr->roll;
    _pitch = ptr->pitch;
    _yaw = ptr->yaw;
}

void Levelness::_PubResult(const std::vector<double>& result)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"initialize and publish a cloud.");
        return;
    }
    auto msg = std::make_unique<shared_interfaces::msg::Float64Array>();
    msg -> data = result;
    _pubResult->publish(std::move(msg));

    RCLCPP_INFO(this->get_logger(), "%s publish result: %f.", GetLocalTime(), result[0]);
}


void Levelness::_PubStatus(const std::string& status)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"initialize and publish a status.");
        return; 
    }

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = status;
    _pubStatus->publish(std::move(msg));

    RCLCPP_INFO(this->get_logger(), "%s publish status: %s.", GetLocalTime(), status.c_str());
}

}//namespace am

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(am::Levelness)