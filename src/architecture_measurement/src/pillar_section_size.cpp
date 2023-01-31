#include "architecture_measurement/pillar_section_size.hpp"

#include "architecture_measurement/impl/file_io.hpp"
#include "architecture_measurement/impl/space_analytic_geometry.hpp"
#include "architecture_measurement/impl/cloud_pretreatment.hpp"
#include "architecture_measurement/impl/cloud_header.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/ModelCoefficients.h"

#include "opencv2/core.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgproc.hpp"

#include "nlohmann/json.hpp"

#include <chrono>
#include <memory>
#include <algorithm>
#include <fstream>
#include <vector>
#include <numeric>

#define TEST

using namespace std::chrono_literals;

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

bool SortVerticalBoundaryXL(VerticalBoundary& a, VerticalBoundary& b)
{
    return a.GetX() < b.GetX();
}

bool SortVerticalBoundaryXG(VerticalBoundary& a, VerticalBoundary& b)
{
    return a.GetX() > b.GetX();
}

bool SortVerticalBoundaryYL(VerticalBoundary& a, VerticalBoundary& b)
{
    return a.GetY() < b.GetY();
}

bool SortVerticalBoundaryYG(VerticalBoundary& a, VerticalBoundary& b)
{
    return a.GetY() > b.GetY();
}

bool SortVerticalBoundaryZL(VerticalBoundary& a, VerticalBoundary& b)
{
    return a.GetZ() < b.GetZ();
}

bool SortVerticalBoundaryZG(VerticalBoundary& a, VerticalBoundary& b)
{
    return a.GetZ() > b.GetZ();
}

bool SortLengthL(VerticalBoundary& a, VerticalBoundary& b)
{
    return a.GetLength() < b.GetLength();
}

bool SortLengthG(VerticalBoundary& a, VerticalBoundary& b)
{
    return a.GetLength() > b.GetLength();
}

bool SortPointCloudZL(const pcl::PointXYZI& point1, const pcl::PointXYZI& point2)
{
    return point1.z < point2.z;
}

bool SortPointCloudZG(const pcl::PointXYZI& point1, const pcl::PointXYZI& point2)
{
    return point1.z > point2.z;
}

VerticalBoundary::VerticalBoundary(pcl::ModelCoefficients _coef, pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud)
{
    coef = _coef;
    cloud = _cloud;

    // std::cout <<"VerticalBoundary cloud size: " << cloud ->size() << std::endl;

    CalculateLineLength(); 
    ClaculateLineCenter();   
    CalculateLimitsZ();  
}

VerticalBoundary::~VerticalBoundary()
{
    cloud.reset();
    // std::cout << "destrucotr successful." <<std::endl;
}

void VerticalBoundary::CalculateLineLength()
{
    std::sort(cloud->begin(), cloud->end(), SortPointCloudZL);
    pcl::PointXYZI point_binary_1 = *(cloud->begin());
    pcl::PointXYZI point_binary_2 = *(cloud->end()-1);
    Eigen::Vector3f point_binary_vector_1{point_binary_1.x, point_binary_1.y, point_binary_1.z};
    Eigen::Vector3f point_binary_vector_2{point_binary_2.x, point_binary_2.y, point_binary_2.z};
    Eigen::Vector3f vector = point_binary_vector_2 - point_binary_vector_1;
    length = vector.norm();

    // std::cout << "length: " << length << std::endl;
}

void VerticalBoundary::ClaculateLineCenter()
{
    std::sort(cloud->begin(), cloud->end(), SortPointCloudZL);
    pcl::PointXYZI point_binary_1 = *(cloud->begin());
    pcl::PointXYZI point_binary_2 = *(cloud->end()-1);
    Eigen::Vector3f point_binary_vector_1{point_binary_1.x, point_binary_1.y, point_binary_1.z};
    Eigen::Vector3f point_binary_vector_2{point_binary_2.x, point_binary_2.y, point_binary_2.z};
    Eigen::Vector3f center = (point_binary_vector_1 + point_binary_vector_2) / 2.;
    x = center(0);
    y = center(1);
    z = center(2);
    // std::cout << "x: " << x << std::endl;
    // std::cout << "y: " << y << std::endl;
    // std::cout << "z: " << z << std::endl;
}

void VerticalBoundary::CalculateLimitsZ()
{
    std::sort((cloud->points).begin(), (cloud->points).end(), SortPointCloudZL);//将竖直边界点云按照Z坐标从小到大进行排序
    (cloud->points).erase((cloud->points).begin(), (cloud->points).begin() + 3);
    (cloud->points).erase((cloud->points).end() - 3, (cloud->points).end());//竖直边界两端部分点

    limit_z_s = (cloud->points).begin() -> z;//左侧竖直边界Z坐标最小值
    limit_z_l = (cloud->points).rbegin() -> z;//左侧竖直边界Z坐标最大值

    // std::cout << "limit_z_s: " << limit_z_s <<std::endl;
    // std::cout << "limit_z_l: " << limit_z_l << std::endl;
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

class PillarSectionSize::_Impl
{
    public:
        explicit _Impl(PillarSectionSize * ptr) : _node(ptr)
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

            _node->declare_parameter<double>("extraParam",0.0);

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

            _node->get_parameter("extraParam",extraParam);
            _node->get_parameter("engineering", _node->_engineeringVec);
            _node->get_parameter("option", _node->_optionVec); 
        }

        void SortCloud(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clouds, int& index)
        {
            CloudHeaderVector cds(clouds.size());

            for(size_t i = 0; i < clouds.size(); i++)
            {
                CloudHeader cd(clouds[i], i);
                cds[i] = cd;
            }

            std::cout << "cds size: " << cds.size() << std::endl;
            //筛选竖直平面
            auto cds_end = std::remove_if(cds.begin(),cds.end(),[](CloudHeader cd){return std::fabs(cd.GetNormalVector().dot(Eigen::Vector3d::UnitZ())) > THRESHOLD;});
            cds.erase(cds_end, cds.end());
            if(cds.empty())
            {
                std::string error = "Error in SortCloud(): cds is empty(after remove horizontal plane).";                                
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }
            std::cout << "cds size (remove horizontal plane): " << cds.size() << std::endl;

            cds_end = std::remove_if(cds.begin(),cds.end(),[](CloudHeader cd){return std::fabs(cd.GetNormalVector().dot(Eigen::Vector3d::UnitX())) < THRESHOLD;});
            cds.erase(cds_end, cds.end());
            if(cds.empty())
            {
                std::string error = "Error in SortCloud(): cds is empty(after remove XOZ).";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }
            std::cout << "cds size (remove parallel XOZ): " << cds.size() << std::endl;

            //筛选距离世界水平坐标系YOZ平面最近的竖直平面
            std::sort(cds.begin(),cds.end(),SortMassCenterXL);

            index = cds[0].GetIndex();
        }

        double CalculateDistance(const pcl::PointCloud<pcl::PointXYZI>::Ptr& line1,  pcl::ModelCoefficients line2_coeff)
        {
            std::vector<double> result;
            result.reserve(line1->points.size());

            for(auto& point : *line1)
            {
                Eigen::Vector3d vec1{point.x-line2_coeff.values[0],point.y-line2_coeff.values[1],point.z-line2_coeff.values[2]};
                Eigen::Vector3d vec2{line2_coeff.values[3],line2_coeff.values[4],line2_coeff.values[5]};

                double dis=(vec1.cross(vec2)).norm()/vec2.norm();
                result.push_back(dis);       
            }

            double sum = std::accumulate(result.begin(), result.end(), 0.0);
            double distance = sum / result.size();
            distance += 2;
            distance /= 1000.;
            // std::vector<double> diff(result.size());
            // std::transform(result.begin(), result.end(), diff.begin(), [mean](double x) { return x - mean; });
            // auto dev = std::sqrt(std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) / result.size());    

            // double minD,maxD;
            // auto extremum=std::minmax_element(result.begin(),result.end());
            // minD = *extremum.first;
            // maxD = *extremum.second;

            return distance;
        }

        std::vector<double> Calculate(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const int& object) try
        {
            if(cloud -> points.empty())
            {
                std::string error = "Error in Calculate(): cloud is empty.";
                std::string statusNumber = "-1";
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

            //降采样
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel_grid_filter(new pcl::PointCloud<pcl::PointXYZI>);
            cp.VoxelGridFilter(cloud,cloud_voxel_grid_filter);
            if(cloud_voxel_grid_filter -> points.size() < 0)
            {
                std::string error = "Error in Calculate():cloud_voxel_grid_filter is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //半径滤波
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_radius_filter(new pcl::PointCloud<pcl::PointXYZI>);
            cp.RadiusFilter(cloud_voxel_grid_filter,cloud_radius_filter);
            if(cloud_radius_filter -> points.size() < 0)
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
            pcl::PointCloud<pcl::Normal>::Ptr cloud_transform_normal(new pcl::PointCloud<pcl::Normal>);
            cp.NormalEstimation(cloud_transform,cloud_transform_normal);

            //平面分割
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
            cp.PalneSegmentation(cloud_transform, cloud_transform_normal, clouds);
            if(clouds.empty())
            {
                std::string error = "Error in Calculate(): there is no region.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //筛选平面
            int index = -1;
            SortCloud(clouds, index);
            if(index < 0)
            {
                std::string error = "Error in Calculate(): index is invalid.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //提取最佳平面点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_optimal(clouds[index]);
#ifdef TEST
            FileIO ofile;
            std::string name;
            name = "/home/ubuntu/tmp/pillar_section_size/cloud_origin.txt";
            ofile.SetProperty(name);
            ofile.Write(cloud_optimal);

            name = "/home/ubuntu/tmp/pillar_section_size/cloud_optimal.txt";
            ofile.SetProperty(name);
            ofile.Write(cloud_optimal);
#endif //TEST

            //边界提取
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);//点云法向量
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZI>);//点云平面边界            

            pcl::PointCloud<pcl::PointXYZI>::Ptr line(new pcl::PointCloud<pcl::PointXYZI>);//竖直方向上的直线
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_remain(new pcl::PointCloud<pcl::PointXYZI>);//其余点

            cp.NormalEstimation(cloud_optimal, cloud_normal);//估计法向量
            cp.Boundary(cloud_optimal,cloud_normal,cloud_boundary);//提取点云边界
#ifdef TEST
            name = "/home/ubuntu/tmp/pillar_section_size/cloud_boundary.txt";
            ofile.SetProperty(name);
            ofile.Write(cloud_boundary);
#endif //TEST

            std::vector<VerticalBoundary> verticalBoundaries;//竖直边界
            //提取全部可能的竖直边界
            while(true)
            {
                pcl::PointIndices::Ptr inliers_line(new pcl::PointIndices);//竖直方向上点的索引

                int status = cp.ParallelLine(cloud_boundary,inliers_line,Eigen::Vector3f::UnitZ(), 100);//拟合竖直边界，并计算参数,数值边界的点数量应不小于100
                if(status < 0)
                    break;
                cp.Extract(cloud_boundary,inliers_line,line,false);//提取竖直边界
                pcl::PointIndices::Ptr line_indices(new pcl::PointIndices);//竖直方向上点的索引
                pcl::ModelCoefficients parallel_line_coeff = cp.Line(line, line_indices);
                cp.Extract(cloud_boundary,inliers_line,cloud_remain,true);//提取其余点
                std::swap(cloud_boundary->points, cloud_remain->points);//将剩余点与边界点进行交换

                VerticalBoundary vb(parallel_line_coeff, line);//构造竖直边界
                verticalBoundaries.emplace_back(vb);       
            }
            std::cout << "verticalBoundaries size: " << verticalBoundaries.size() <<std::endl;
            if(verticalBoundaries.empty())
            {
                std::string error = "Error in Calculate(): verticalBoundaries is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }
             
            //将竖直边界按照边界的长短进行排序
            auto verticalBoundaries_end = std::remove_if(verticalBoundaries.begin(), verticalBoundaries.end(), [](VerticalBoundary& vb)
            {
                return vb.GetLength() < 500;
            });
            verticalBoundaries.erase(verticalBoundaries_end, verticalBoundaries.end());            

            //判断竖直边界的数量
            if(verticalBoundaries.size() < 2)
            {
                std::string error = "Error in Calculate(), vertical boundary is invalid.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            std::sort(verticalBoundaries.begin(), verticalBoundaries.end(), SortVerticalBoundaryYL);//按照中心点Y坐标从小到大的顺序对竖直边界进行排序

            VerticalBoundary vertical_boundary_left = *verticalBoundaries.rbegin();//选择最左边的一条竖直边界构造竖直边缘对象
            VerticalBoundary vertical_boundary_right = *verticalBoundaries.begin();//构造竖直边缘对象

            float line_left_z_limit_s = vertical_boundary_left.limit_z_s;//左侧竖直边界Z坐标最小值
            float line_left_z_limit_l = vertical_boundary_left.limit_z_l;//左侧竖直边界Z坐标最大值

            float line_right_z_limit_s = vertical_boundary_right.limit_z_s;//右侧竖直边界Z坐标最小值
            float line_right_z_limit_l = vertical_boundary_right.limit_z_l;//右侧竖直边界Z坐标最大值
#ifdef TEST
            std::cout << "line_left_z_limit_s: " << line_left_z_limit_s << std::endl;
            std::cout << "line_left_z_limit_l: " << line_left_z_limit_l << std::endl;
            std::cout << "line_right_z_limit_s: " << line_right_z_limit_s << std::endl;
            std::cout << "line_right_z_limit_l: " << line_right_z_limit_l << std::endl;
#endif //TEST

            //计算竖直边界在Z方向上的相交范围
            float z_limit_s = 0;//Z坐标最小值
            float z_limit_l = 0;//Z坐标最大值
            float z_limit_m = 0;//Z坐标中间值
            if(line_left_z_limit_l > line_right_z_limit_l)
            {
                if(line_left_z_limit_s <= line_right_z_limit_s)
                {
                    z_limit_s = line_right_z_limit_s;
                    z_limit_l = line_right_z_limit_l;
                }
                else if(line_left_z_limit_s > line_right_z_limit_s && line_left_z_limit_s <= line_right_z_limit_l)
                {
                    z_limit_s = line_left_z_limit_s;
                    z_limit_l = line_right_z_limit_l;
                }
                else if(line_left_z_limit_s > line_right_z_limit_l)
                {
                    std::string error = "Error in Calculate(): condition 1: there is no intersection.";
                    std::string statusNumber = "-1";
                    _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                    throw std::runtime_error(error);
                }
            }
            else if(line_left_z_limit_l <= line_right_z_limit_l)
            {
                if(line_right_z_limit_s <= line_left_z_limit_s)
                {
                    z_limit_s = line_left_z_limit_s;
                    z_limit_l = line_left_z_limit_l;
                }
                else if(line_right_z_limit_s > line_left_z_limit_s && line_right_z_limit_s <= line_left_z_limit_l)
                {
                    z_limit_s = line_right_z_limit_s;
                    z_limit_l = line_left_z_limit_l;
                }
                else if(line_right_z_limit_s > line_left_z_limit_l)
                {
                    std::string error = "Error in Calculate(): condition 2: there is no intersection.";
                    std::string statusNumber = "-1";
                    _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                    throw std::runtime_error(error);
                }
            }

            z_limit_m = (z_limit_s + z_limit_l)/2.0;
#ifdef TEST
            std::cout << "z_limit_l: " << z_limit_l << std::endl;
            std::cout << "z_limit_m: " << z_limit_m << std::endl;
            std::cout << "z_limit_s: " << z_limit_s << std::endl;
#endif //TEST

            //计算靠尺平面，筛选靠尺点云，计算截面尺寸
            std::vector<double> result;//截面尺寸结果
            pcl::PointCloud<pcl::PointXYZI>::Ptr rulers(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云数据
            for(int i = -2; i < 3; i++)
            {
                //virtual ruler
                float binary_lower = z_limit_m + 2 * i * 50 - 25;//靠尺平面下边界
                float binary_upper = z_limit_m + 2 * i * 50 + 25;//靠尺平面上边界
                if(binary_lower < z_limit_s || binary_upper > z_limit_l)
                {
                    result.push_back(-1);
                    continue;
                }
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ruler(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云
                cp.ConditionRemoval(cloud_optimal, cloud_ruler, am::AXIS_Z, binary_lower, binary_upper);//筛选靠尺点云
                if(cloud_ruler -> size() < 100)
                {
                    result.push_back(-1);
                    continue;
                }
                *rulers += *cloud_ruler;

                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ruler_boundary(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云边界

                pcl::PointCloud<pcl::Normal>::Ptr cloud_ruler_normal(new pcl::PointCloud<pcl::Normal>);//靠尺点云法向量
                cp.NormalEstimation(cloud_ruler, cloud_ruler_normal);//估计靠尺点云法向量
                cp.Boundary(cloud_ruler,cloud_ruler_normal,cloud_ruler_boundary);//提取靠尺点云边界

#ifdef TEST
                FileIO ofile;
                name = "/home/ubuntu/tmp/pillar_section_size/cloud_ruler_boundary_" + std::to_string(i) + ".txt";
                ofile.SetProperty(name);
                ofile.Write(cloud_ruler_boundary);
#endif //TEST

                pcl::ModelCoefficients plane_coeff = cp.Plane(cloud_ruler_boundary);//使用靠尺点云边界进行平面拟合

                Eigen::Vector3f cloud_ruler_boundary_normal;//靠尺点云边界法矢
                cloud_ruler_boundary_normal << plane_coeff.values[0], plane_coeff.values[1], plane_coeff.values[2];
                if(cloud_ruler_boundary_normal(2) < 0)
                    cloud_ruler_boundary_normal = -cloud_ruler_boundary_normal;

                //将靠尺点云边界法矢转到世界水平坐标系Z轴方向
                Eigen::Vector3f rotate_axis = cloud_ruler_boundary_normal.cross(Eigen::Vector3f::UnitZ());//旋转轴
                float rotate_angle = std::acos(cloud_ruler_boundary_normal.dot(Eigen::Vector3f::UnitZ()));//旋转角

                Eigen::AngleAxisf rotate{rotate_angle,rotate_axis};
                Eigen::Matrix3f rotate_matrix = rotate.matrix();

                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ruler_boundary_transform(new pcl::PointCloud<pcl::PointXYZI>);
                Eigen::Affine3f ruler_transform = Eigen::Affine3f::Identity();
                ruler_transform.translation() << 0.0, 0.0, 0.0;
                ruler_transform.rotate (rotate_matrix);
                pcl::transformPointCloud (*cloud_ruler_boundary, *cloud_ruler_boundary_transform, ruler_transform);

                //世界水平坐标系XOY平面参数
                pcl::ModelCoefficients::Ptr XOY_plane_coef(new pcl::ModelCoefficients);
                XOY_plane_coef->values.resize(4);
                XOY_plane_coef->values[0] = 0.;
                XOY_plane_coef->values[1] = 0.;
                XOY_plane_coef->values[2] = 1.;
                XOY_plane_coef->values[3] = 0.;

                //将转换后的靠尺边界点云投影到世界水平坐标系的XOY平面
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ruler_projected(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::ProjectInliers<pcl::PointXYZI> proj;
                proj.setModelType(pcl::SACMODEL_PLANE);
                proj.setInputCloud(cloud_ruler_boundary_transform);
                proj.setModelCoefficients(XOY_plane_coef);
                proj.filter(*cloud_ruler_projected);

                //将投影后的靠尺边界点云
                std::vector<cv::Point> points2D;
                for(auto& p : *cloud_ruler_projected)
                {
                    cv::Point point;
                    point.x = std::round(p.x);
                    point.y = std::round(p.y);
                    points2D.push_back(point);
                }

                cv::RotatedRect rect = cv::minAreaRect(points2D);                
                auto width = rect.size.width;
                auto height = rect.size.height;
#ifdef TEST
                std::cout << "rect width: " << width << std::endl;
                std::cout << "rect height: " << height << std::endl;
#endif //TEST
                result.push_back(width > height ? width + extraParam : height + extraParam);              
            }

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
            FileIO fo(fileName);
            fo.Write(ground_cloud_transform);

            // FileIO fo(fileName);
            // fo.Write(cloud_optimal);

            //测试部分
#ifdef TEST
            // std::string time = GetLocalTime();
            std::string point_cloud_origin_name = "/home/ubuntu/tmp/pillar_section_size/pointcloud_origin.txt";
            std::string point_cloud_transform_name = "/home/ubuntu/tmp/pillar_section_size/pointcloud_transform.txt";
            std::string point_cloud_optimal_name = "/home/ubuntu/tmp/pillar_section_size/pointcloud_optimal.txt";
            std::string point_cloud_ruler_name = "/home/ubuntu/tmp/pillar_section_size/pointcloud_ruler.txt";

            fo.SetProperty(point_cloud_origin_name);
            fo.Write(cloud_voxel_grid_filter);

            fo.SetProperty(point_cloud_transform_name);
            fo.Write(cloud_transform);          

            fo.SetProperty(point_cloud_optimal_name);
            fo.Write(cloud_optimal);

            fo.SetProperty(point_cloud_ruler_name);
            fo.Write(rulers);  

            for(auto& r : result)
            {
                std::cout << "result: " << r << std::endl;
            }
#endif //TEST


            return result;
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

        double extraParam;

        Eigen::Matrix3d I_R_C;

        std::string fileName;

        PillarSectionSize* _node;
};



/**** PillarSectionSize ****/
PillarSectionSize::PillarSectionSize(const rclcpp::NodeOptions& option = rclcpp::NodeOptions()) : rclcpp::Node("pillar_section_size_node", option)
{
    _initThread = std::thread(&PillarSectionSize::_Init, this);
}

PillarSectionSize::~PillarSectionSize()try
{
    _initThread.join();

    _subPointCloud.reset();
    _subRpy.reset();
    _pubResult.reset();
    _pubStatus.reset();

    RCLCPP_INFO(this->get_logger(),"%s PillarSectionSize destroyed successfully.", GetLocalTime());
}
catch(const std::exception& e)
{
    RCLCPP_INFO(this->get_logger(), "%s Exception in PillarSectionSize destruction: %s", GetLocalTime(), e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_INFO(this->get_logger(),"%s Exception in PillarSectionSize destruction: unknown", GetLocalTime());
    rclcpp::shutdown();
}

void PillarSectionSize::_Init() try
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
        std::bind(&PillarSectionSize::_SubPointCloud, this, std::placeholders::_1));

    _subRpy = this->create_subscription<shared_interfaces::msg::RollPitchYaw>(
        _subRpyName, 
        10, 
        std::bind(&PillarSectionSize::_SubRpy, this, std::placeholders::_1)
    );

    _pubResult = this->create_publisher<shared_interfaces::msg::Float64Array>(_pubResultName, 1);

    _pubStatus = this->create_publisher<std_msgs::msg::String>(_pubStatusName, 1);

    _status = 0;

    RCLCPP_INFO(this->get_logger(), "%s PillarSectionSize initialized successfully.", GetLocalTime());
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(this->get_logger(), "%s Exception in PillarSectionSize initializer: %s", GetLocalTime(), e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(this->get_logger(),"%s Exception in PillarSectionSize initializer: unknown", GetLocalTime());
    rclcpp::shutdown();
}

void PillarSectionSize::_InitializeParameters()
{
    return;
}

void PillarSectionSize::_UpdateParameters()
{
    return;
}

void PillarSectionSize::_SubPointCloud(shared_interfaces::msg::PointCloudC::UniquePtr pointCloudCPtr)
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

void PillarSectionSize::_SubRpy(shared_interfaces::msg::RollPitchYaw::UniquePtr ptr)
{
    if(_status < 0)
    {
        return;
    }

    _roll = ptr->roll;
    _pitch = ptr->pitch;
    _yaw = ptr->yaw;
}

void PillarSectionSize::_PubResult(const std::vector<double>& result)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"initialize and publish result");
        return;
    }
    auto msg = std::make_unique<shared_interfaces::msg::Float64Array>();
    msg -> data = result;
    _pubResult->publish(std::move(msg));

    RCLCPP_INFO(this->get_logger(), "%s publish result: %f.", GetLocalTime(), result[0]);
}

void PillarSectionSize::_PubStatus(const std::string& status)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"initialize and publish status");
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
RCLCPP_COMPONENTS_REGISTER_NODE(am::PillarSectionSize)