#include "architecture_measurement/internal_and_external_angle.hpp"

#include "architecture_measurement/impl/file_io.hpp"
#include "architecture_measurement/impl/space_analytic_geometry.hpp"
#include "architecture_measurement/impl/cloud_header.hpp"
#include "architecture_measurement/impl/cloud_pretreatment.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <fstream>

#include "pcl/ModelCoefficients.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/transforms.h"

#include "nlohmann/json.hpp"

#define TEST

namespace am
{

using namespace std::chrono_literals;
using json = nlohmann::json;

//向量夹角阈值
constexpr double THRESHOLD_ANGLE = std::sqrt(2)/2.;

//点到线的距离阈值
constexpr double THRESHOLD_LINE_DIST = 50;

//水平坐标系Z轴
const Eigen::Vector3d UNIT_Z{0., 0., 1.};

//水平坐标系XOY平面
const Plane3d PLANE_XOY{UNIT_Z(0),UNIT_Z(1),UNIT_Z(2),0};

//向量起始点
const Point3d LINE_ORIGIN_POINT{0.0, 0.0, 0.0};

//绕世界水平坐标系下Z轴旋转90度的旋转矩阵
const Eigen::Matrix3d ROTATE_Z = Eigen::AngleAxisd( M_PI/2., UNIT_Z).matrix();

//获取当前时间点
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

//按照Z坐标，从小到大对点进行排序
bool SortPointCloudZL(pcl::PointXYZI& p1, pcl::PointXYZI& p2)
{
    return p1.z < p2.z;
}

//按照Z坐标，从达到小对点进行排序
bool SortPointCloudZG(pcl::PointXYZI& p1, pcl::PointXYZI& p2)
{
    return p1.z > p2.z;
}

//按照直线上一点X坐标到原点的距离，从小到大对直线进行排序
bool SortIntersectionLineXC(IntersectionLine& line1, IntersectionLine& line2)
{
    auto point1 = line1.point;
    auto point2 = line2.point;
    return std::fabs(point1(0)) < std::fabs(point2(0));
}

//按照直线上一点Y坐标到原点的距离，从小到大对直线进行排序
bool SortIntersectionLineYC(IntersectionLine& line1, IntersectionLine& line2)
{
    auto point1 = line1.point;
    auto point2 = line2.point;
    return std::fabs(point1(1)) < std::fabs(point2(1));
}

//按照直线上一点Z坐标到原点的距离，从小到大对直线进行排序
bool SortIntersectionLineZC(IntersectionLine& line1, IntersectionLine& line2)
{
    auto point1 = line1.point;
    auto point2 = line2.point;
    return std::fabs(point1(2)) < std::fabs(point2(2));
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

class InternalAndExternalAngle::_Impl
{
    public:
        explicit _Impl(InternalAndExternalAngle * ptr) : _node(ptr)
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
            _node->declare_parameter<double>("distanceThread", 50.);
            _node->declare_parameter<std::string>("fileName","file");
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

            _node->get_parameter("distanceThread", distanceThread);

            _node->get_parameter("engineering", _node->_engineeringVec);
            _node->get_parameter("option", _node->_optionVec); 
        }

        void SortCloud(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clouds, IntersectionLine& line, const int& type)
        {       
            //cloudHeader对象
            CloudHeaderVector cds(clouds.size());

            for(std::size_t i = 0; i < cds.size(); i++)
            {
                CloudHeader cd(clouds[i], i);
                cds[i] = cd;
            }

            //筛选出竖直平面，删除水平平面
            //水平平面特征：平面法矢 与 竖直Z轴 之间的点乘 的 绝对值 大于阈值
            auto cds_end = std::remove_if(cds.begin(),cds.end(),[&UNIT_Z](CloudHeader cd){return std::fabs(cd.GetNormalVector().dot(UNIT_Z)) > THRESHOLD_ANGLE;});
            cds.erase(cds_end, cds.end());
            if(cds.empty())
            {
                std::string error = "Error in SortCloud(): cds is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }
#ifdef TEST
            std::cout << "cds size (remove horizontal): " << cds.size() << std::endl;

            for(auto iter = cds.begin();  iter != cds.end(); iter++)
            {
                auto normal = (*iter).GetNormalVector();
                auto id = iter - cds.begin() + 1;
                RCLCPP_INFO(_node->get_logger(), "%s vertical plane[%d] normal: %f, %f, %f", GetLocalTime(), id, normal(0), normal(1), normal(2));
            }
#endif //TEST

            //将竖直平面按照质心Y坐标从大到小（从左到右）的顺序进行排序
            std::sort(cds.begin(), cds.end(), SortMassCenterYG); 
#ifdef TEST
            for(auto iter = cds.begin(); iter != cds.end(); iter++)
            {
                auto masscenter = (*iter).GetMassCenter();
                auto id = iter - cds.begin() + 1;
                RCLCPP_INFO(_node->get_logger(), "%s vertical plane[%d] masscenter y: %f", GetLocalTime(), id, masscenter(1));
            }
#endif //TEST

            //判断阴阳角
            std::vector<IntersectionPlaneIndices> indices_tmp;//阴阳角平面索引
            for(std::size_t i = 0, j = i + 1;  j < cds.size(); i++,j++)
            {
                std::cout << "i: " << i << "j: " << j << std::endl;
                
                //相邻的两个点云平面
                auto& cd_left = cds[i];
                auto& cd_right = cds[j];

#ifdef TEST
                std::cout << "cd_left.GetPlaneCoeff(): " << cd_left.GetPlaneCoeff().transpose() << std::endl;
                std::cout << "cd_right.GetPlaneCoeff(): " << cd_right.GetPlaneCoeff().transpose() <<std::endl;
#endif //TEST

                //获取相邻两个平面各自的法向量
                Vector3d leftPlaneNormal{cd_left.GetNormalVector()};
                Vector3d rightPlaneNormal{cd_right.GetNormalVector()};

                //根据法向量和向量起始点，构造直线对象
                Line3d leftLine{LINE_ORIGIN_POINT,leftPlaneNormal};
                Line3d rightLine{LINE_ORIGIN_POINT,rightPlaneNormal};

                //将直线投影到水平坐标系下的XOY平面
                LineAndPlane lap1{leftLine,PLANE_XOY};
                Line3d leftProjectLine;
                lap1.GetProjection(leftProjectLine);

                LineAndPlane lap2{rightLine,PLANE_XOY};
                Line3d rightProjectLine;
                lap2.GetProjection(rightProjectLine);

                //将左点云平面的投影直线绕世界水平坐标系Z轴旋转90度
                auto leftRotateVector = ROTATE_Z * leftProjectLine.vector.vector;
                auto rightRotateVector = rightProjectLine.vector.vector;

                //计算两点云平面投影直线之间的夹角
                auto dot_value = leftRotateVector.dot(rightRotateVector);
#ifdef TEST
                std::cout << "dot value: " << dot_value << std::endl;
#endif //TEST
                if(dot_value < 0 && type == INTERNAL_ANGLE)
                {
#ifdef TEST
                    std::cout << "INTERNAL_ANGLE" << std::endl;
#endif //TEST
                    InternalAngleIndices index{i, j};
                    indices_tmp.push_back(index);
                }
                else if(dot_value > 0 && type == EXTERNAL_ANGLE)
                {
#ifdef TEST
                    std::cout << "EXTERNAL_ANGLE" << std::endl;
#endif //TEST
                    ExternalAngleIndices index{i, j};
                    indices_tmp.push_back(index);
                }
                else
                {
                    std::this_thread::sleep_for(200ms);
                    std::string error = "Error in SortCloud(): wrong type.";
                    std::string statusNumber = "-1";
                    std::cout << "error message: " << error << std::endl;
                    _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                    // throw std::runtime_error(error);
                    continue;
                }             
            }

            if(indices_tmp.empty())
            {
                std::string error = "Error in SoutCloud(): indices_tmp is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //计算阴阳角平面之间的交线和范围
            std::vector<IntersectionLine> intersection_lines(indices_tmp.size());

            for(std::size_t i = 0; i < indices_tmp.size(); i++)
            {
                //通过索引获取阴阳角平面
                auto& cd_left = cds[indices_tmp[i].index_1];
                auto& cd_right = cds[indices_tmp[i].index_2];

                //获取阴阳角平面的参数
                const Eigen::Vector4d planeCoeff_left = cd_left.GetPlaneCoeff();
                const Eigen::Vector4d planeCoeff_right = cd_right.GetPlaneCoeff();

                //根据平面参数构造平面对象
                Plane3d plane_left{planeCoeff_left[0], planeCoeff_left[1], planeCoeff_left[2], planeCoeff_left[3]};
                Plane3d plane_right{planeCoeff_right[0], planeCoeff_right[1], planeCoeff_right[2], planeCoeff_right[3]};

                //构造双面对象，求解两平面之间的交线
                DoublePlanes doublePlanes{plane_left, plane_right};
                Line3d intersectionLine;
                doublePlanes.GetIntersectionLine(intersectionLine);

                //构造IntersectionLine对象
                IntersectionLine intersection_line;
                intersection_line.indices.index_1 = cd_left.GetIndex();
                intersection_line.indices.index_2 = cd_right.GetIndex();
                intersection_line.point = intersectionLine.point.point;
                intersection_line.vector = intersectionLine.vector.vector;

                intersection_lines[i] = intersection_line;
            }

            if(intersection_lines.empty())
            {
                std::string error = "Error in SortCloud(): intersection_lines is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //根据阴阳角平面交线上的点的Y坐标到世界水平坐标系XOZ平面之间的距离进行排序，选择距离最小的一组阴阳角
            std::sort(intersection_lines.begin(),intersection_lines.end(),SortIntersectionLineYC);
            line = *(intersection_lines.begin());
        }

        std::vector<double> Calculate(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const int& type) try
        {
            if(cloud->points.empty())
            {
                std::string statusNumber = "-1";
                std::string error = "Error in Calculate(): cloud is empty.";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //倾角仪坐标系和世界水平坐标系之间的位姿变换矩阵
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
            cp.RadiusFilter(cloud_voxel_grid_filter, cloud_radius_filter);

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
            transform.rotate(W_R_C_f);
            pcl::transformPointCloud (*cloud_radius_filter, *cloud_transform, transform);

            //法向量计算
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
            cp.NormalEstimation(cloud_transform,cloud_normal);

            //分割平面
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
            cp.PalneSegmentation(cloud_transform, cloud_normal, clouds);
            if(clouds.size() < 2)
            {
                std::string error = "Error in Calculate(): clouds size is less than 2.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //筛选点云
            IntersectionLine line;
            SortCloud(clouds, line, type); 

            //提取阴阳角平面交线段附近的点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intersection_line_left(new pcl::PointCloud<pcl::PointXYZI>);//交线左侧部分点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intersection_line_right(new pcl::PointCloud<pcl::PointXYZI>);//交线右侧部分点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intersection_line(new pcl::PointCloud<pcl::PointXYZI>);//交线部分点云
            
            auto cloud_left = clouds[line.indices.index_1];//获取阴阳角左侧平面
            auto cloud_right = clouds[line.indices.index_2];//获取阴阳角右侧平面

            //构造交线对象
            Point3d p3d{line.point};//点对象
            Vector3d v3d{line.vector};//向量对象
            Line3d l3d{p3d, v3d};//交线对象

            //提取交线左侧部分点云
            for(const auto& p : *cloud_left)
            {
                Point3d point{p.x, p.y, p.z};
                PointAndLine point_and_line{point, l3d};
                double distance;
                point_and_line.GetDistance(distance);
                if(distance < THRESHOLD_LINE_DIST)
                    cloud_intersection_line_left->points.push_back(p);
            }

            //提取交线右侧部分点云
            for(const auto& p : *cloud_right)
            {
                Point3d point{p.x, p.y, p.z};
                PointAndLine point_and_line{point, l3d};
                double distance;
                point_and_line.GetDistance(distance);
                if(distance < THRESHOLD_LINE_DIST)
                    cloud_intersection_line_right->points.push_back(p);                 
            }
            std::cout << "cloud_intersection_line_left size: " << cloud_intersection_line_left->size() << std::endl;
            std::cout << "cloud_intersection_line_right size: " << cloud_intersection_line_right->size() << std::endl;
            if(cloud_intersection_line_left -> size() < 10 || cloud_intersection_line_right -> size() < 10)
            {
                std::string error = "Error in Calculate(): cloud_intersection_line_left is empty || cloud_intersection_line_right is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //将两部分点云合并，得到交线部分点云         
            *cloud_intersection_line += *cloud_intersection_line_left;
            *cloud_intersection_line += *cloud_intersection_line_right;
            if(cloud_intersection_line->size() < 20)
            {
                std::string error = "Error in Calculate(): cloud_intersection_line is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //将交线段点云按照Z坐标从小到大的顺序排列
            auto& points = cloud_intersection_line->points;
            std::sort(points.begin(),points.end(),SortPointCloudZL);

            //计算交线段Z方向上的范围及中心位置
            line.limit_s = (*points.begin()).z;
            line.limit_l = (*points.rbegin()).z;
            line.limit_m = (line.limit_s + line.limit_l)/2.0;  

            if(line.limit_l - line.limit_s < 500)
            {
                std::string error = "Error in Calculate(): limit_l - limit_s < 500.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //筛选靠尺并计算阴阳角
            std::vector<double> result;//阴阳角结果

            pcl::PointCloud<pcl::PointXYZI>::Ptr rulers(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云
            for(int i = -2; i < 3; i++)
            {
                //计算靠尺范围
                float binary_lower = line.limit_m + 2 * i * 50 - 25;//靠尺范围下限
                float binary_upper = line.limit_m + 2 * i * 50 + 25;//靠尺范围上限
                //提取靠尺点云
                pcl::PointCloud<pcl::PointXYZI>::Ptr ruler_point_cloud_tmp_left(new pcl::PointCloud<pcl::PointXYZI>);//左侧靠尺点云(全长)
                pcl::PointCloud<pcl::PointXYZI>::Ptr ruler_point_cloud_tmp_right(new pcl::PointCloud<pcl::PointXYZI>);//右侧靠尺点云(全长)
                pcl::PointCloud<pcl::PointXYZI>::Ptr ruler_point_cloud_left(new pcl::PointCloud<pcl::PointXYZI>);//左侧靠尺点云
                pcl::PointCloud<pcl::PointXYZI>::Ptr ruler_point_cloud_right(new pcl::PointCloud<pcl::PointXYZI>);//右侧靠尺点云
                cp.ConditionRemoval(cloud_left, ruler_point_cloud_tmp_left, am::AXIS_Z, binary_lower, binary_upper);//提取左侧靠尺点云
                cp.ConditionRemoval(cloud_right, ruler_point_cloud_tmp_right, am::AXIS_Z, binary_lower, binary_upper);//提取右侧靠尺点云
                if(cloud_left -> size() < 100 || cloud_right -> size() < 100)
                    continue;
                //筛选左侧靠尺点云到交线距离小于特定阈值的点
                for(const auto& point : *ruler_point_cloud_tmp_left)
                {
                    Point3d ruler_point{point.x, point.y, point.z};
                    Point3d p3d{line.point};
                    Vector3d c3d{line.vector};
                    Line3d intersection_line{p3d, c3d};
                    PointAndLine point_and_line{ruler_point,intersection_line};
                    double distance;
                    point_and_line.GetDistance(distance);
                    if(distance < distanceThread && distance > 10)
                        ruler_point_cloud_left -> points.push_back(point);
                }
                //筛选右侧靠尺点云到交线距离小于200mm的点
                for(const auto& point : *ruler_point_cloud_tmp_right)
                {
                    Point3d ruler_point{point.x, point.y, point.z};
                    Point3d p3d{line.point};
                    Vector3d c3d{line.vector};
                    Line3d intersection_line{p3d, c3d};
                    PointAndLine point_and_line{ruler_point,intersection_line};
                    double distance;
                    point_and_line.GetDistance(distance);
                    if(distance < distanceThread && distance > 10)
                        ruler_point_cloud_right -> points.push_back(point);
                }

                //靠尺点云数据
                *rulers += *ruler_point_cloud_left;
                *rulers += *ruler_point_cloud_right;
                
                if(ruler_point_cloud_left -> size() < 50 || ruler_point_cloud_right -> size() < 50)
                {
                    continue;
                }

                //计算靠尺点云平面参数
                pcl::ModelCoefficients ruler_coeff_left = cp.Plane(ruler_point_cloud_left);//左靠尺平面参数
                pcl::ModelCoefficients ruler_coeff_right = cp.Plane(ruler_point_cloud_right);//右靠尺平面参数
                //计算阴阳角
                Eigen::Vector3f ruler_normal_vector_in_world_left{ruler_coeff_left.values[0],ruler_coeff_left.values[1],ruler_coeff_left.values[2]};//左靠尺平面法矢
                Eigen::Vector3f ruler_normal_vector_in_world_right{ruler_coeff_right.values[0],ruler_coeff_right.values[1],ruler_coeff_right.values[2]};//右靠尺平面法矢
                if(ruler_normal_vector_in_world_left(0) < 0)//左靠尺平面法矢方向验证（与水平坐标系X轴同向）
                    ruler_normal_vector_in_world_left = -ruler_normal_vector_in_world_left;
                if(ruler_normal_vector_in_world_right(0) < 0)//右靠尺平面法矢方向验证（与水平坐标系X轴同向）
                    ruler_normal_vector_in_world_right = -ruler_normal_vector_in_world_right;

                double angle_radian = std::acos(ruler_normal_vector_in_world_left.dot(ruler_normal_vector_in_world_right));//计算两个平面法矢之间的夹角
                double angle_degree = std::fabs((180 - Degree(angle_radian)) - 90);//计算阴阳角与90的偏差
                double angle_dist;
                if(type == EXTERNAL_ANGLE)
                {
                    angle_dist= angle_degree / 0.5;
                    // angle_dist /= 4;
                } 
                else if(type == INTERNAL_ANGLE)
                {
                    angle_dist= angle_degree / 0.5;
                    // angle_dist /= 4;
                }
                else
                {
                    std::string error = "Error in Calculate(): wrong type.";
                    std::string statusNumber = "-1";
                    _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                    throw std::runtime_error(error);
                }

                RCLCPP_INFO(_node->get_logger(), "%s angle: %f deg -- %f mm", GetLocalTime(), angle_degree, angle_dist);

                result.push_back(angle_dist);
            }
            

            //保存数据
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_left_save(clouds[line.indices.index_1]);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_right_save(clouds[line.indices.index_2]);    

            for(auto& p : *cloud_left_save)
            {
                p.intensity = 5;
            }

            for(auto& p : *cloud_right_save)
            {
                p.intensity = -5;
            }

            for(auto& p : *rulers)
            {
                p.intensity = 0;
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_optimal(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr file_pointCloud(new pcl::PointCloud<pcl::PointXYZI>);
            *cloud_optimal += *cloud_left_save;
            *cloud_optimal += *cloud_right_save;
            *file_pointCloud += *cloud_optimal;
            *file_pointCloud += *rulers;

            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_transform(new pcl::PointCloud<pcl::PointXYZI>);
            cp.PointcloudTransform(*file_pointCloud, *ground_cloud_transform, 0);

            //保存点云数据用于显示
            FileIO fo(fileName);
            fo.Write(ground_cloud_transform);

            //测试部分
#ifdef TEST
            std::string time = GetLocalTime();
            std::string point_cloud_origin_name = "/home/ubuntu/tmp/internal_and_external_angle/pointcloud_origin_" + time + ".txt";
            std::string point_cloud_transform_name = "/home/ubuntu/tmp/internal_and_external_angle/pointcloud_transform_" + time + ".txt";
            std::string point_cloud_optimal_name = "/home/ubuntu/tmp/internal_and_external_angle/pointcloud_optimal_" + time + ".txt";
            std::string point_cloud_ruler_name = "/home/ubuntu/tmp/internal_and_external_angle/pointcloud_ruler_" + time + ".txt";

            fo.SetProperty(point_cloud_origin_name);
            fo.Write(cloud_voxel_grid_filter);

            fo.SetProperty(point_cloud_transform_name);
            fo.Write(cloud_transform);          

            fo.SetProperty(point_cloud_optimal_name);
            fo.Write(cloud_optimal);

            fo.SetProperty(point_cloud_ruler_name);
            fo.Write(rulers);
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
        Eigen::Matrix3d I_R_C;
        double distanceThread;
        std::string fileName;

        InternalAndExternalAngle* _node;
};

/**** InternalAndExternalAngle ****/

InternalAndExternalAngle::InternalAndExternalAngle(const rclcpp::NodeOptions& option = rclcpp::NodeOptions()) : Node("internal_and_external_node", option)
{
    _initThread = std::thread(&InternalAndExternalAngle::_Init,this);
}

InternalAndExternalAngle::~InternalAndExternalAngle()try
{
    _initThread.join();

    _subPointCloud.reset();
    _subRpy.reset();
    _pubResult.reset();
    _pubStatus.reset();

    RCLCPP_INFO(this->get_logger(),"%s InternalAndExternalAngle destroyed successfully.", GetLocalTime());
}
catch(const std::exception& e)
{
    RCLCPP_INFO(this->get_logger(), "%s Exception in InternalAndExternalAngle destruction: %s", GetLocalTime(), e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_INFO(this->get_logger(),"%s Exception in InternalAndExternalAngle destruction: unknown", GetLocalTime());
    rclcpp::shutdown();
}

void InternalAndExternalAngle::_Init() try
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
        std::bind(&InternalAndExternalAngle::_SubPointCloud, this, std::placeholders::_1));

    _subRpy = this->create_subscription<shared_interfaces::msg::RollPitchYaw>(
        _subRpyName, 
        10, 
        std::bind(&InternalAndExternalAngle::_SubRpy, this, std::placeholders::_1)
    );

    _pubResult = this->create_publisher<shared_interfaces::msg::Float64Array>(_pubResultName, 1);

    _pubStatus = this->create_publisher<std_msgs::msg::String>(_pubStatusName, 1);

    _status = 0;

    RCLCPP_INFO(this->get_logger(), "%s InternalAndExternalAngle initialized successfully.", GetLocalTime());
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(this->get_logger(), "%s Exception in FloorHeight initializer: %s", GetLocalTime(), e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(this->get_logger(),"%s Exception in FloorHeight initializer: unknown",  GetLocalTime());
    rclcpp::shutdown();
}

void InternalAndExternalAngle::_InitializeParameters()
{
    return;
}

void InternalAndExternalAngle::_UpdateParameters()
{
    return;
}

void InternalAndExternalAngle::_SubPointCloud(shared_interfaces::msg::PointCloudC::UniquePtr pointCloudCPtr)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"initialize and subscribe pointcloud.");
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

void InternalAndExternalAngle::_SubRpy(shared_interfaces::msg::RollPitchYaw::UniquePtr ptr)
{
    if(_status < 0)
    {
        return;
    }
    _roll = ptr->roll;
    _pitch = ptr->pitch;
    _yaw = ptr->yaw;
}

void InternalAndExternalAngle::_PubResult(const std::vector<double>& result)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "initialize and publish a result.");
        return;
    }
    auto msg = std::make_unique<shared_interfaces::msg::Float64Array>();
    msg -> data = result;
    _pubResult->publish(std::move(msg));

    RCLCPP_INFO(this->get_logger(), "%s publish result: %f.", GetLocalTime(), result[0]);
}

void InternalAndExternalAngle::_PubStatus(const std::string& status)
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
RCLCPP_COMPONENTS_REGISTER_NODE(am::InternalAndExternalAngle)