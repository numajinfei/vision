#include "architecture_measurement/surface_flatness.hpp"

#include "architecture_measurement/impl/space_analytic_geometry.hpp"
#include "architecture_measurement/impl/cloud_pretreatment.hpp"
#include "architecture_measurement/impl/file_io.hpp"
#include "architecture_measurement/impl/cloud_header.hpp"

#include "pcl/ModelCoefficients.h"
#include "pcl/common/transforms.h"
#include "pcl/sample_consensus/sac_model_plane.h"
#include "pcl_conversions/pcl_conversions.h"

#include "Eigen/Geometry"

#include "nlohmann/json.hpp"

#include <thread>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <numeric>

#define TEST

namespace am
{
using namespace std::chrono_literals;
using json = nlohmann::json;

// constexpr double THRESHOLD = std::sqrt(2)/2.0;

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

class SurfaceFlatness::_Impl
{
    public:
        explicit _Impl(SurfaceFlatness * ptr) : _node(ptr)
        {
            _InitializeParameters();
            _UpdateParameters();
        }

        ~_Impl()
        {

        }

        void _InitializeParameters()
        {
            _node->declare_parameter<std::vector<std::string>>("engineering", {"Normal"});
            _node->declare_parameter<std::vector<std::string>>("option",{"StoreyHeight"});
            _node->declare_parameter<std::vector<double>>("I_R_C",{0.});
            _node->declare_parameter<std::string>("fileName","file");
        }

        void _UpdateParameters()
        {
            std::vector<double> I_R_C_vector;         
            _node->get_parameter("I_R_C",I_R_C_vector);

            double* I_R_C_vector_ptr = I_R_C_vector.data();
            I_R_C = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> >(I_R_C_vector_ptr);

            _node->get_parameter("fileName",fileName);

            _node->get_parameter("engineering", _node->_engineeringVec);
            _node->get_parameter("option", _node->_optionVec); 
        }



        void SortCloud(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clouds, const int& type, int& index)
        {
            //点云头
            CloudHeaderVector cds(clouds.size());
            for(std::size_t i = 0; i < clouds.size(); i++)
            {        
                CloudHeader cd{clouds[i], i};
                cds[i] = cd;
            }

            //删除地面和顶板点云
            if(type == 0)
            {
                auto cds_end = std::remove_if(cds.begin(),cds.end(),
                    [](CloudHeader cd)
                    {
#ifdef TEST
                        std::cout << "cd.GetNormalVector(): " << cd.GetNormalVector().transpose() << std::endl;
                        std::cout << "Eigen::Vector3d::UnitZ(): " << Eigen::Vector3d::UnitZ().transpose() << std::endl;
                        std::cout << "dot value: " << std::fabs( cd.GetNormalVector().dot( Eigen::Vector3d::UnitZ() ) ) << std::endl;
                        // std::cout << "THRESHOLD: " << THRESHOLD << std::endl;
                        std::cout << "dot_value < THRESHOLD: " << bool(std::fabs(cd.GetNormalVector().dot(Eigen::Vector3d::UnitZ())) < THRESHOLD) << std::endl;
#endif //TEST
                        bool bool_value = std::fabs(cd.GetNormalVector().dot(Eigen::Vector3d::UnitZ())) > THRESHOLD;
                        return bool_value;
                    });
                cds.erase(cds_end, cds.end());
                if(cds.empty())
                {                    
                    std::string error = "Error in SortCloud():  cds (remove horizontal plane) is empty.";
                    std::string statusNumber = "-1";
                    _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));
                    
                    throw std::runtime_error(error);   
                }
            }
            //删除侧墙点云
            else
            {
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
                    std::string error = "Error in SortCloud():  cds (remove vertical plane) is empty";
                    std::string statusNumber = "-1";
                    _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                    throw std::runtime_error(error);   
                }

                //删除顶板
                if(type == 1)
                {
                    cds_end = std::remove_if(cds.begin(),cds.end(),
                        [](CloudHeader cd)
                        {
#ifdef TEST
                            std::cout << "cd.GetMassCenter()(2): " << cd.GetMassCenter()(2) << std::endl;
                            std::cout << "cd.GetMassCenter()(2) > 0: " << bool(cd.GetMassCenter()(2) > 0) << std::endl;
#endif //TEST
                            return cd.GetMassCenter()(2) > 0;
                        });
                    cds.erase(cds_end, cds.end());
                    if(cds.empty())
                    {
                        std::string error = "Error in SortCloud():  cds size (remove roof plane) < 1.";
                        std::string statusNumber = "-1";
                        _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                        throw std::runtime_error(error);   
                    }
                }
                //删除地面
                else if(type == 2)
                {
                    cds_end = std::remove_if(cds.begin(),cds.end(),
                        [](CloudHeader cd)
                        {
#ifdef TEST
                            std::cout << "cd.GetMassCenter()(2): " << cd.GetMassCenter()(2) << std::endl;
                            std::cout << "cd.GetMassCenter()(2) < 0: " << bool(cd.GetMassCenter()(2) < 0) << std::endl;
#endif //TEST
                            return cd.GetMassCenter()(2) < 0;
                        });//去除地面
                    cds.erase(cds_end, cds.end());
                    if(cds.empty())
                    {
                        std::string error = "Error in SortCloud():  cds size (remove ground plane) < 1.";
                        std::string statusNumber = "-1";
                        _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                        throw std::runtime_error(error);   
                    }
                }
                else
                {
                    std::string error = "Error in SortCloud(): type is invalid.";
                    std::string statusNumber = "-1";
                    _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                    throw std::runtime_error(error);
                }
            }

            std::sort(cds.begin(), cds.end(), SortSize);
            index = cds[0].GetIndex();

            return;
        }

        std::vector<double> Calculate(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const int& type) try
        {
            if(cloud->points.empty())
            {
                std::string statusNumber = "-1";
                std::string error = "Error in Calculate(): cloud is empty.";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            time_t now = time(0);
            std::tm* localTime = std::localtime(&now);
            std::string time_ptr = "[" + std::to_string(1900 + localTime->tm_year) + "-" 
                + std::to_string(1 + localTime->tm_mon) + "-"
                + std::to_string(localTime->tm_mday) + " "
                + std::to_string(localTime->tm_hour) + ":"
                + std::to_string(localTime->tm_min) + ":"
                + std::to_string(localTime->tm_sec) + "]";

            FileIO _ofile;
            std::string fileName = "/home/ubuntu/tmp/pointcloud_";
            // std::string time = GetLocalTime();
            std::string fileType = ".txt";
            fileName = fileName + time_ptr + fileType;

            //_ofile.SetProperty(fileName);
            //_ofile.Write(cloud);

            //倾角仪坐标系和世界水平坐标系之间的变换矩阵
            double roll_radian = Radian(_node->_roll);
            double pitch_radian = Radian(_node->_pitch);
            double yaw_radian = Radian(_node->_yaw);
            
            Eigen::Matrix3d W_R_I = GetMatrixFromRPY(roll_radian, pitch_radian, yaw_radian);
            Eigen::Matrix3d W_R_C = W_R_I * I_R_C;
            Eigen::Matrix3f W_R_C_f = W_R_C.cast<float>();

            //点云预处理对象
            CloudPretreatment cp;    

            //点云降采样
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel_grid_filter(new pcl::PointCloud<pcl::PointXYZI>);
            cp.VoxelGridFilter(cloud, cloud_voxel_grid_filter);
            if(cloud_voxel_grid_filter->points.empty())
            {
                std::string statusNumber = "-1";
                std::string error = "Error in Calculate(): cloud_voxel_grid_filter is empty.";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //半径滤波
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_radius_filter(new pcl::PointCloud<pcl::PointXYZI>);
            cp.RadiusFilter(cloud_voxel_grid_filter,cloud_radius_filter);
            if(cloud_radius_filter->points.empty())
            {
                std::string statusNumber = "-1";
                std::string error = "Error in Calculate(): cloud_radius_filter is empty.";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //点云坐标变换
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transform(new pcl::PointCloud<pcl::PointXYZI>);
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.translation() << 0.0, 0.0, 0.0;
            transform.rotate (W_R_C_f);
            pcl::transformPointCloud (*cloud_radius_filter, *cloud_transform, transform);

#ifdef TEST
            //保存原始降采样点云
            FileIO fo;
            std::string cloudFileName = "/home/ubuntu/tmp/surface_flatness/origin.txt";
            fo.SetProperty(cloudFileName);
            fo.Write(cloud_voxel_grid_filter);

            //保存变换点云
            cloudFileName = "/home/ubuntu/tmp/surface/transform.txt";
            fo.SetProperty(cloudFileName);
            fo.Write(cloud_transform);
#endif //TEST
            //计算法向量
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
            cp.NormalEstimation(cloud_transform, cloud_normal);

            //区域生长分割平面
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
            cp.PalneSegmentation(cloud_transform, cloud_normal, clouds);
            if(clouds.empty())
            {
                std::string statusNumber = "-1";
                std::string error = "Error in Calculate(): clouds is empty.";    
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //筛选平面点云
            int index = -1;
            SortCloud(clouds, type, index);

            //extract
            if(index < 0)
            {
                std::string statusNumber = "-1";
                std::string error = "Error in Calculate(): index < 0.";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_optimal(clouds[index]);

            auto plane_coeff = cp.Plane(cloud_optimal);
            for(auto& p : *cloud_optimal)
                // p.intensity = pcl::pointToPlaneDistanceSigned<pcl::PointXYZI>(p, plane_coeff.values[0], plane_coeff.values[1], plane_coeff.values[2], plane_coeff.values[3]);
                p.intensity = 5;

            // Eigen::Vector3d plane_normal, palne_normal_ori;
            // plane_normal << plane_coeff.values[0], plane_coeff.values[1], plane_coeff.values[2];
            // palne_normal_ori = W_R_C.inverse() * plane_normal;

            // std::string AI_data_file = "/home/ubuntu/tmp/AI_data.txt";
            // std::ofstream ofile(AI_data_file, std::ofstream::app);
            // if(!ofile.is_open())
            //     return VirtualRuler(cloud_optimal);
            
            // ofile << palne_normal_ori(0) << " " <<  palne_normal_ori(1) << " " <<  palne_normal_ori(2) << "    "
            //         << _node->_roll << " " << _node->_pitch << " " << _node->_yaw << std::endl;

            // ofile.close();

            return VirtualRuler(cloud_optimal);
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO(_node->get_logger(),"%s %s.",GetLocalTime(),e.what());
            return {-1,-1,-1,-1};
        }
        catch(...)
        {
            RCLCPP_INFO(_node->get_logger(),"%s unknown error.",GetLocalTime());
            return {-1,-1,-1,-1};
        }

        std::vector<double> VirtualRuler(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
        {
            //倾角仪数据
            double roll_radian = Radian(_node->_roll);
            double pitch_radian = Radian(_node->_pitch);
            double yaw_radian = Radian(_node->_yaw);

            //相机坐标系到世界水平坐标系的变换矩阵
            Eigen::Matrix3d W_R_I = GetMatrixFromRPY(roll_radian, pitch_radian, yaw_radian);
            Eigen::Matrix3d W_R_C = W_R_I * I_R_C;
            Eigen::Matrix3f W_R_C_f = W_R_C.cast<float>();

            //点云处理对象
            CloudPretreatment cp;

            Eigen::Vector3d plane_horizontal_in_camera{0.,1.,0.};//相机坐标系Y轴矢量
            Eigen::Vector3d plane_vertical_in_camera{1.,0.,0.};//相机坐标系X轴矢量
            Eigen::Vector3d plane_left_falling_in_camera{std::sqrt(0.5), std::sqrt(0.5), 0.};//相机坐标系X轴和Y轴角平分线向量
            Eigen::Vector3d plane_right_falling_in_camera{std::sqrt(0.5), -std::sqrt(0.5), 0.};//相机坐标系X轴和-Y轴角平分线向量

            Eigen::Vector3d plane_horizontal_in_world = W_R_C * plane_horizontal_in_camera;//相机坐标系Y轴矢在世界水平坐标系下的矢量
            Eigen::Vector3d plane_vertical_in_world = W_R_C * plane_vertical_in_camera;//相机坐标系X轴矢量在世界水平坐标系下的矢量
            Eigen::Vector3d plane_left_falling_in_world = W_R_C * plane_left_falling_in_camera;//相机坐标系X轴和Y轴角平分线向量在世界水平坐标系下的矢量
            Eigen::Vector3d plane_right_falling_in_world = W_R_C * plane_right_falling_in_camera;//相机坐标系X轴和-Y轴角平分线向量在世界水平坐标系下的矢量

            pcl::PointCloud<pcl::PointXYZI>::Ptr line0(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云--横
            pcl::PointCloud<pcl::PointXYZI>::Ptr line1(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云--竖
            pcl::PointCloud<pcl::PointXYZI>::Ptr line2(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云--撇
            pcl::PointCloud<pcl::PointXYZI>::Ptr line3(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云--捺
            pcl::PointIndices::Ptr line0_indices(new pcl::PointIndices);//靠尺点云索引--横
            pcl::PointIndices::Ptr line1_indices(new pcl::PointIndices);//靠尺点云索引--竖
            pcl::PointIndices::Ptr line2_indices(new pcl::PointIndices);//靠尺点云索引--撇
            pcl::PointIndices::Ptr line3_indices(new pcl::PointIndices);//靠尺点云索引--捺

            //提取靠尺点云
            for (uint32_t i = 0; i < cloud->width; ++i) 
            {
                //三维点转化为向量
                const auto & point_in_cloud = cloud->at(i);
                Eigen::Vector3d point;
                point << point_in_cloud.x, point_in_cloud.y, point_in_cloud.z;

                double distance = -1;//点到平面的距离

                //筛选靠尺点云--横
                distance = std::fabs(point.dot(plane_horizontal_in_world));
                if(distance < 12.5)
                line0_indices->indices.push_back(i);

                //筛选靠尺点云--竖
                distance = std::fabs(point.dot(plane_vertical_in_world));
                if(distance < 12.5)
                line1_indices->indices.push_back(i);

                //筛选靠尺点云--撇
                distance = std::fabs(point.dot(plane_left_falling_in_world));
                if(distance < 12.5)
                line2_indices->indices.push_back(i);

                //筛选靠尺点云--捺
                distance = std::fabs(point.dot(plane_right_falling_in_world));
                if(distance < 12.5)
                line3_indices->indices.push_back(i);
            }

            //提取靠尺点云--横，并计算平整度
            if(line0_indices->indices.empty())
            {
                std::string error = "Error in VirtualRuler(): line0_indices is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }
            cp.Extract(cloud, line0_indices, line0, false);
            std::pair<double, double> s0 = Statistic(line0);

            //提取靠尺点云--竖，并计算平整度
            if(line1_indices->indices.empty())
            {
                std::string error = "Error in VirtualRuler(): line0_indices is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));
                throw std::runtime_error(error);
            }
            cp.Extract(cloud, line1_indices, line1, false);
            std::pair<double, double> s1 = Statistic(line1);

            //提取靠尺点云--撇，并计算平整度
            if(line2_indices->indices.empty())
            {
                std::string error = "Error in VirtualRuler(): line0_indices is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }
            cp.Extract(cloud, line2_indices, line2, false);
            std::pair<double, double> s2 = Statistic(line2);

            //提取靠尺点云--捺，并计算平整度
            if(line3_indices->indices.empty())
            {
                std::string error = "Error in VirtualRuler(): line0_indices is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }
            cp.Extract(cloud, line3_indices, line3, false);
            std::pair<double, double> s3 = Statistic(line3);

            //四条靠尺点云合并
            pcl::PointCloud<pcl::PointXYZI>::Ptr ruler_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
            *ruler_pointcloud += *line0;
            *ruler_pointcloud += *line1;
            *ruler_pointcloud += *line2;
            *ruler_pointcloud += *line3;

            //四条靠尺点云索引合并
            pcl::PointIndices all_ruler_indices;
            all_ruler_indices.indices.insert(all_ruler_indices.indices.begin(), (line0_indices->indices).begin(), (line0_indices->indices).end());
            all_ruler_indices.indices.insert(all_ruler_indices.indices.begin(), (line1_indices->indices).begin(), (line1_indices->indices).end());
            all_ruler_indices.indices.insert(all_ruler_indices.indices.begin(), (line2_indices->indices).begin(), (line2_indices->indices).end());
            all_ruler_indices.indices.insert(all_ruler_indices.indices.begin(), (line3_indices->indices).begin(), (line3_indices->indices).end());

            //提取背景点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointIndices::Ptr all_ruler_indices_ptr(new pcl::PointIndices(all_ruler_indices));
            if(all_ruler_indices_ptr->indices.empty())
            {
                std::string error = "Error in VirtualRuler(): all_ruler_indices is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }
            cp.Extract(cloud, all_ruler_indices_ptr, ground_cloud, true);

            //靠尺点云强度值设为-10
            for(auto& p : *ruler_pointcloud)
            {
                p.intensity = 0;
            }

            for(auto& p : *ground_cloud)
            {
                p.intensity = 5;
            }

            //背景点云和靠尺点云合并
            *ground_cloud += *ruler_pointcloud;

            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_transform(new pcl::PointCloud<pcl::PointXYZI>);
            cp.PointcloudTransform(*ground_cloud, *ground_cloud_transform);

            //保存点云数据用于显示
            FileIO fo(fileName);
            fo.Write(ground_cloud_transform);

#ifdef TEST
            //保存靠尺点云--横
            std::string cloudFileName;
            cloudFileName = "/home/ubuntu/tmp/surface/line0.txt";
            fo.SetProperty(cloudFileName);
            fo.Write(line0);

            //保存靠尺点云--竖
            cloudFileName = "/home/ubuntu/tmp/surface/line1.txt";
            fo.SetProperty(cloudFileName);
            fo.Write(line1);

            //保存靠尺点云--撇
            cloudFileName = "/home/ubuntu/tmp/surface/line2.txt";
            fo.SetProperty(cloudFileName);
            fo.Write(line2);
            
            //保存靠尺点云--捺
            cloudFileName = "/home/ubuntu/tmp/surface/line3.txt";
            fo.SetProperty(cloudFileName);
            fo.Write(line3);

            //保存合并的靠尺点云
            cloudFileName = "/home/ubuntu/tmp/surface/line_all.txt";
            fo.SetProperty(cloudFileName);
            fo.Write(ruler_pointcloud);
#endif //TEST

            return {s0.first, s1.first, s2.first, s3.first};
        }


        std::pair<double, double> Statistic(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
        {
            //判断点云数据是否为空
            if(cloud->points.empty())
            {
                std::string error = "Error in Statistic(): cloud is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }                

            //点云预处理对象
            CloudPretreatment cp;

            //拟合平面
            pcl::ModelCoefficients plane_coeff = cp.Plane(cloud);

            //计算点到平面的有向距离
            std::vector<double> distanceSigned;
            distanceSigned.reserve(cloud->points.size());
            for (auto & p : *cloud) 
            {
                double distance = pcl::pointToPlaneDistanceSigned<pcl::PointXYZI>(
                    p,
                    plane_coeff.values[0],
                    plane_coeff.values[1],
                    plane_coeff.values[2],
                    plane_coeff.values[3]);

                distanceSigned.push_back(distance);
            }

            //计算点到平面的距离极差
            auto result = std::minmax_element (distanceSigned.begin(),distanceSigned.end());
            double distance = *result.second - *result.first;

            return {distance, 0};
        }

    private:
        Eigen::Matrix3d I_R_C;
        std::string fileName;

        SurfaceFlatness* _node;
};

/**** SurfaceFlatness ****/
SurfaceFlatness::SurfaceFlatness(const rclcpp::NodeOptions& option = rclcpp::NodeOptions()) : rclcpp::Node("surface_flatness_node", option)
{
    _initThread = std::thread(&SurfaceFlatness::_Init, this);
}

SurfaceFlatness::~SurfaceFlatness()try
{
    _initThread.join();

    _subPointCloud.reset();
    _subRpy.reset();
    _pubResult.reset();

    RCLCPP_INFO(this->get_logger(),"%s SurfaceFlatness destroyed successfully.", GetLocalTime());
}
catch(const std::exception& e)
{
    RCLCPP_INFO(this->get_logger(), "%s Exception in SurfaceFlatness destruction: %s", GetLocalTime(), e.what());
}
catch(...)
{
    RCLCPP_INFO(this->get_logger(),"%s Exception in SurfaceFlatness destruction: unknown", GetLocalTime());
}

void SurfaceFlatness::_Init() try
{
    _InitializeParameters();
    _UpdateParameters();

    _roll = 0;
    _pitch = 0;
    _yaw = 0;

    _status = -1;

    _impl = std::make_unique<_Impl>(this);

    _subPointCloud = this->create_subscription<shared_interfaces::msg::PointCloudC>(
        _subPointCloudName, 
        1, 
        std::bind(&SurfaceFlatness::_SubPointCloud, this, std::placeholders::_1));

    _subRpy = this->create_subscription<shared_interfaces::msg::RollPitchYaw>(
        _subRpyName, 
        10, 
        std::bind(&SurfaceFlatness::_SubRpy, this, std::placeholders::_1)
    );

    _pubResult = this->create_publisher<shared_interfaces::msg::Float64Array>(_pubResultName, 1);

    _pubStatus = this->create_publisher<std_msgs::msg::String>(_pubStatusName, 1);

    _status = 0;

    RCLCPP_INFO(this->get_logger(), "%s SurfaceFlatness initialized successfully.", GetLocalTime());
}
catch(const std::exception& e)
{
    RCLCPP_INFO(this->get_logger(), "%s Exception in SurfaceFlatness initializer: %s", GetLocalTime(), e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_INFO(this->get_logger(),"%s Exception in SurfaceFlatness initializer: unknown", GetLocalTime());
    rclcpp::shutdown();
}

void SurfaceFlatness::_InitializeParameters()
{
    return;
}

void SurfaceFlatness::_UpdateParameters()
{
    return;
}

void SurfaceFlatness::_SubPointCloud(shared_interfaces::msg::PointCloudC::UniquePtr pointCloudCPtr) try
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"initialize and subscribe a point cloud.");
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

    for(int i = 0; i < point_cloud_number; i++)
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
catch(const std::exception& e)
{
    RCLCPP_INFO(this->get_logger(), "%s %s", GetLocalTime(), e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_INFO(this->get_logger(),"%s Exception in _SubPointCloud(): unknown", GetLocalTime());
    rclcpp::shutdown();
}

void SurfaceFlatness::_SubRpy(shared_interfaces::msg::RollPitchYaw::UniquePtr ptr) try
{
    if(_status < 0)
    {
        return;
    }

    _roll = ptr->roll;
    _pitch = ptr->pitch;
    _yaw = ptr->yaw;
}
catch(const std::exception& e)
{
    RCLCPP_INFO(this->get_logger(), "%s %s", GetLocalTime(), e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_INFO(this->get_logger(),"%s Exception in _SubRpy(): unknown", GetLocalTime());
    rclcpp::shutdown();
}

void SurfaceFlatness::_PubResult(const std::vector<double>& result)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "initialize and publish a result.");
        return;
    }
    auto msg = std::make_unique<shared_interfaces::msg::Float64Array>();
    msg -> data = result;
    _pubResult->publish(std::move(msg));

    RCLCPP_INFO(this->get_logger(), "%s publish result: %f, %f, %f, %f.", GetLocalTime(), result[0], result[1], result[2], result[3]);
}

void SurfaceFlatness::_PubStatus(const std::string& status)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "initialize and publish a status.");
        return;
    }

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = status;
    _pubStatus->publish(std::move(msg));

    RCLCPP_INFO(this->get_logger(), "%s publish status: %s.", GetLocalTime(), status.c_str());
}

}//am

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(am::SurfaceFlatness)