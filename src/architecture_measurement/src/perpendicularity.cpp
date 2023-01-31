#include "architecture_measurement/perpendicularity.hpp"

#include "architecture_measurement/impl/file_io.hpp"
#include "architecture_measurement/impl/space_analytic_geometry.hpp"
#include "architecture_measurement/impl/cloud_pretreatment.hpp"
#include "architecture_measurement/impl/cloud_header.hpp"

#include "pcl/ModelCoefficients.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/transforms.h"

#include "nlohmann/json.hpp"

#include <chrono>
#include <fstream>
#include <cmath>

#define TEST

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


VerticalBoundary::VerticalBoundary() {}

VerticalBoundary::VerticalBoundary(
    pcl::ModelCoefficients& coeffs, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr)
{
    _coeffs = coeffs;
    _pointCloudPtr = pointCloudPtr;

    CalculateLineLength(); 
    ClaculateLineCenter();   
    CalculateLimitsZ();  
}

VerticalBoundary::~VerticalBoundary()
{
    _pointCloudPtr.reset();
}

void VerticalBoundary::CalculateLineLength()
{
    std::sort(_pointCloudPtr->begin(), _pointCloudPtr->end(), SortPointCloudZL);
    pcl::PointXYZI pointBundary1 = *(_pointCloudPtr->begin());
    pcl::PointXYZI pointBundary2 = *(_pointCloudPtr->end()-1);
    Eigen::Vector3f pointBundaryVec1{pointBundary1.x, pointBundary1.y, pointBundary1.z};
    Eigen::Vector3f pointBundaryVec2{pointBundary2.x, pointBundary2.y, pointBundary2.z};
    Eigen::Vector3f vector = pointBundaryVec2 - pointBundaryVec1;
    _length = vector.norm();
}

void VerticalBoundary::ClaculateLineCenter()
{
    std::sort(_pointCloudPtr->begin(), _pointCloudPtr->end(), SortPointCloudZL);
    pcl::PointXYZI pointBundary1 = *(_pointCloudPtr->begin());
    pcl::PointXYZI pointBundary2 = *(_pointCloudPtr->end()-1);
    Eigen::Vector3f pointBundaryVec1{pointBundary1.x, pointBundary1.y, pointBundary1.z};
    Eigen::Vector3f pointBundaryVec2{pointBundary2.x, pointBundary2.y, pointBundary2.z};
    Eigen::Vector3f center = (pointBundaryVec1 + pointBundaryVec2) / 2.;
    _x = center(0);
    _y = center(1);
    _z = center(2);
}

void VerticalBoundary::CalculateLimitsZ()
{
    auto points = _pointCloudPtr->points;
    std::sort(points.begin(), points.end(), SortPointCloudZL);//将竖直边界点云按照Z坐标从小到大进行排序
    points.erase(points.begin(), points.begin() + 3);
    points.erase(points.end() - 3, points.end());//竖直边界两端部分点

    _limitZS = points.begin() -> z;//左侧竖直边界Z坐标最小值
    _limitZL = points.rbegin() -> z;//左侧竖直边界Z坐标最大值
}


bool SortVerticalBoundaryXL(VerticalBoundary& a, VerticalBoundary& b)
{
    return a._x < b._x;
}

bool SortVerticalBoundaryXG(VerticalBoundary& a, VerticalBoundary& b)
{
    return a._x > b._x;
}

bool SortVerticalBoundaryYL(VerticalBoundary& a, VerticalBoundary& b)
{
    return a._y < b._y;
}

bool SortVerticalBoundaryYG(VerticalBoundary& a, VerticalBoundary& b)
{
    return a._y > b._y;
}

bool SortVerticalBoundaryZL(VerticalBoundary& a, VerticalBoundary& b)
{
    return a._z < b._z;
}

bool SortVerticalBoundaryZG(VerticalBoundary& a, VerticalBoundary& b)
{
    return a._z > b._z;
}

bool SortLengthL(VerticalBoundary& a, VerticalBoundary& b)
{
    return a._length < b._length;
}

bool SortLengthG(VerticalBoundary& a, VerticalBoundary& b)
{
    return a._length > b._length;
}

bool SortPointCloudXL(pcl::PointXYZI& point1, pcl::PointXYZI& point2)
{
    return point1.x < point2.x;
}

bool SortPointCloudXG(pcl::PointXYZI& point1, pcl::PointXYZI& point2)
{
    return point1.x > point2.x;
}

bool SortPointCloudYL(pcl::PointXYZI& point1, pcl::PointXYZI& point2)
{
    return point1.y < point2.y;
}

bool SortPointCloudYG(pcl::PointXYZI& point1, pcl::PointXYZI& point2)
{
    return point1.y > point2.y;
}

bool SortPointCloudZL(pcl::PointXYZI& point1, pcl::PointXYZI& point2)
{
    return point1.z < point2.z;
}

bool SortPointCloudZG(pcl::PointXYZI& point1, pcl::PointXYZI& point2)
{
    return point1.z > point2.z;
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

class Perpendicularity::_Impl
{
    public:
        explicit _Impl(Perpendicularity * ptr) : _node(ptr)
        {
            _InitializeParameters();
            _UpdateParameters();
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


        int SortCloud(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clouds,const int& type, int& index)
        {   
            //cloudHeader对象
            CloudHeaderVector cds(clouds.size());

            for(int i = 0; i < clouds.size(); i++)
            {        
                CloudHeader cd{clouds[i], i};
                cds[i] = cd;
            }

#ifdef TEST
            std::cout << "cds size (init): " << cds.size() << std::endl;
#endif //TEST

            //祛除水平面
            auto cds_end = std::remove_if(cds.begin(),cds.end(),[](CloudHeader cd){return std::fabs(cd.GetNormalVector().dot(Eigen::Vector3d::UnitZ())) > THRESHOLD;});
            cds.erase(cds_end, cds.end());
            if(cds.empty())
            {
                std::string error = "Error in SortCloud(): cds is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }
#ifdef TEST
            std::cout << "cds size (remove horizontal plane): " << cds.size() << std::endl;
#endif //TEST
            switch(type)
            {
                case SIDEWALL://侧墙
                {                    
                    std::sort(cds.begin(), cds.end(), SortSize);//按照点云的数量从大到小进行排序
                    index = cds[0].GetIndex();//选择数量最多的点云平面
                    break;
                }
                case PILLAR://柱
                {
                    auto cds_end = std::remove_if(cds.begin(),cds.end(),[](CloudHeader cd){return std::fabs(cd.GetNormalVector().dot(Eigen::Vector3d::UnitX())) < THRESHOLD;});
                    cds.erase(cds_end, cds.end());
                    if(cds.empty())
                    {
                        std::string error = "Error in SortCloud(): cds is empty.";
                        std::string statusNumber = "-1";
                        _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                        throw std::runtime_error(error);
                    }
                    std::sort(cds.begin(),cds.end(),SortMassCenterXL);//按照点云质心X坐标从小到大进行排序
                    index = cds[0].GetIndex();//质心X坐标最小的点云平面
                    break;
                }
                default:
                {                    
                    std::string error = "Error in SortCloud(): type is invalid.";
                    std::string statusNumber = "-1";
                    _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                    throw std::runtime_error(error);
                }
            }
            return 0;
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
#ifdef TEST
            std::cout << "roll: " << _node->_roll << std::endl;
            std::cout << "pitch: " << _node->_pitch << std::endl;
            std::cout << "yaw: " << _node->_yaw << std::endl;
#endif //TEST

            double roll_radian = Radian(_node->_roll);
            double pitch_radian = Radian(_node->_pitch);
            double yaw_radian = Radian(_node->_yaw);

            
            Eigen::Matrix3d W_R_I = GetMatrixFromRPY(roll_radian, pitch_radian, yaw_radian);
            Eigen::Matrix3d W_R_C = W_R_I * I_R_C;
            Eigen::Matrix3f W_R_C_f = W_R_C.cast<float>();
            // std::cout << "I_R_C: \n" << I_R_C << std::endl;

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
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.translation() << 0.0, 0.0, 0.0;
            transform.rotate (W_R_C_f);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transform(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud (*cloud_radius_filter, *cloud_transform, transform);

            //法向量估计
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
            cp.NormalEstimation(cloud_transform,cloud_normal);

            //区域生长分割平面
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
            cp.PalneSegmentation(cloud_transform, cloud_normal, clouds);
            if(clouds.empty())
            {
                std::string error = "Error in Calculate(): clouds is empty.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            //区分平面
            int index = -1;
            SortCloud(clouds, type, index);

            //提取被测平面点云
            if(index < 0)
            {
                std::string error = "Error in Calculate(): index is invalid.";
                std::string statusNumber = "-1";
                _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                throw std::runtime_error(error);
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_optimal(clouds[index]);//最优点云平面
            pcl::ModelCoefficients plane_coeff = cp.Plane(cloud_optimal);//最优点云平面进行平面拟合，获取平面参数

            std::vector<double> result;//垂直度结果
            pcl::PointCloud<pcl::PointXYZI>::Ptr rulers(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云

            switch (type)
            {
                case SIDEWALL:  
                {
                    Eigen::Vector3d camera_axisZ{0.,0.,1.};//相机坐标系Z轴
                    Eigen::Vector3d camera_axisZ_in_world = W_R_C * camera_axisZ;//相机坐标系Z轴在水平坐标系下的矢量
                    Eigen::Vector3d origin_point{0., 0., 0.};//原点坐标

                    Line3d line{origin_point, camera_axisZ_in_world};//线对象
                    Plane3d plane{plane_coeff.values[0],plane_coeff.values[1],plane_coeff.values[2],plane_coeff.values[3]};//最优点云平面对象
                    std::cout << "plane normal:  " <<  plane_coeff.values[0] << " " << plane_coeff.values[1] << " " << plane_coeff.values[2] << std::endl;
                    LineAndPlane line_and_plane{line, plane};//线和平面对象
                    Point3d intersection_point;//点对象
                    line_and_plane.GetIntersectionPoint(intersection_point);//计算点和平面之间的交点
                    auto center_Y = intersection_point.point(1);//求解交点y坐标

                    for(int i = -2; i < 3; i++)
                    {
                        float binary_left = center_Y + 2 * i * 50 - 25;//靠尺边界下边缘
                        float binary_right = center_Y + 2 * i * 50 + 25;//靠尺边界上边缘
                        pcl::PointCloud<pcl::PointXYZI>::Ptr ruler_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云
                        cp.ConditionRemoval(cloud_optimal, ruler_point_cloud, am::AXIS_Y, binary_left, binary_right);//筛选靠尺点云
                        if(ruler_point_cloud -> size() < 100)
                        {
                            result.push_back(-1);
                            continue;
                        }
                            
                        pcl::ModelCoefficients ruler_coeff = cp.Plane(ruler_point_cloud);//靠尺点云拟合平面
                        Eigen::Vector3d ruler_normal_vector_in_world{ruler_coeff.values[0],ruler_coeff.values[1],ruler_coeff.values[2]};//靠尺点云平面法矢
                        std::cout << "ruler_normal_vector_in_world :" << ruler_normal_vector_in_world.transpose() << std::endl;
                        *rulers += *ruler_point_cloud;

                        double angle_radian = std::acos(Eigen::Vector3d::UnitZ().dot(ruler_normal_vector_in_world));//计算靠尺平面法矢与水平坐标系Z轴之间的夹角
                        angle_radian = std::fabs(angle_radian - M_PI / 2.);
                        double angle_degree = Degree(angle_radian);//计算夹角与90度之间的偏差              
                        
                        std::sort(ruler_point_cloud->points.begin(), ruler_point_cloud->points.end(), SortPointCloudZL);
                        double ruler_distance = std::fabs( (*ruler_point_cloud->points.begin()).z - (*ruler_point_cloud->points.rbegin()).z );

                        auto one_result = ruler_distance * std::tan(angle_radian) /2.0;

                        std::cout <<"angle_degree: " <<  angle_degree << std::endl;
                        std::cout << "ruler_distance: " << ruler_distance << "extraParam: " << extraParam << std::endl;
                        std::cout << "one_result: " << one_result << std::endl;

                        result.push_back(one_result);                 

                    }
                    break;
                }
                case PILLAR:
                {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZI>);//点云平面边界
                    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);//点云法向量
                    
                    pcl::PointCloud<pcl::PointXYZI>::Ptr line(new pcl::PointCloud<pcl::PointXYZI>);//竖直方向上的直线
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_remain(new pcl::PointCloud<pcl::PointXYZI>);//其余点

                    cp.NormalEstimation(cloud_optimal, cloud_normal);//估计法向量
                    cp.Boundary(cloud_optimal,cloud_normal,cloud_boundary);//提取点云边界

                    std::vector<VerticalBoundary> verticalBoundaries;//竖直边界
                    //提取全部可能的竖直边界
                    while(true)
                    {
                            pcl::PointIndices::Ptr inliers_line(new pcl::PointIndices);//竖直方向上点的索引
                            int status = cp.ParallelLine(cloud_boundary,inliers_line,Eigen::Vector3f::UnitZ(), 100);//拟合竖直边界，并计算参数
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

                    if(verticalBoundaries.size() < 2)
                    {
                        std::string error = "Error in Calculate(): verticalBoundaries size is zero.";
                        std::string statusNumber = "-1";
                        _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                        throw std::runtime_error(error);
                    }
                    //将竖直边界按照边界的长短进行排序
                    auto verticalBoundaries_end = std::remove_if(verticalBoundaries.begin(), verticalBoundaries.end(), [](VerticalBoundary& vb)
                    {
                        return vb._length < 500;
                    });
                    verticalBoundaries.erase(verticalBoundaries_end, verticalBoundaries.end());

                    //判断竖直边界的数量
                    if(verticalBoundaries.size() < 2)
                    {
                        std::string error = " Error in Calculate(), vertical boundary is invalid.";
                        std::string statusNumber = "-1";
                        _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                        throw std::runtime_error(error);
                    }

                    std::sort(verticalBoundaries.begin(), verticalBoundaries.end(), SortVerticalBoundaryYL);//按照中心点Y坐标从小到大的顺序对竖直边界进行排序

                    VerticalBoundary vertical_boundary_left = *verticalBoundaries.rbegin();//选择最左边的一条竖直边界构造竖直边缘对象
                    VerticalBoundary vertical_boundary_right = *verticalBoundaries.begin();//构造竖直边缘对象

                    float y_limit_l = vertical_boundary_left._y;//左侧竖直边界Y坐标
                    float y_limit_s = vertical_boundary_right._y;//右侧竖直边界Y坐标
                    //计算竖直边界在Z方向上的相交范围
                    if(y_limit_l - y_limit_s < 100)
                    {
                        std::string error = " Error in Calculate(), pillar section size < 100mm.";
                        std::string statusNumber = "-1";
                        _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                        throw std::runtime_error(error);
                    }

                    float y_limit_m = (y_limit_s + y_limit_l)/2.0;

#ifdef TEST
                    std::cout << "y_limit_s: " << y_limit_s << std::endl;
                    std::cout << "y_limit_l: " << y_limit_l << std::endl;
                    std::cout << "y_limit_m: " << y_limit_m << std::endl;
#endif //TEST

                    //计算两个竖直边界与中心水平面之间的交点，及交点之间的方向向量
                    Plane3d planeXOY{0., 0., 1., 0.};

                    auto line_coeff_left = vertical_boundary_left._coeffs.values;//左竖直边界参数
                    auto line_coeff_right = vertical_boundary_right._coeffs.values;//右竖直边界参数

                    Eigen::Vector3d line_point_left{line_coeff_left[0],line_coeff_left[1],line_coeff_left[2]};//左竖直边界点
                    Eigen::Vector3d line_point_right{line_coeff_right[0],line_coeff_right[1],line_coeff_right[2]};//右竖直边界点
                    Eigen::Vector3d line_vector_left{line_coeff_left[3],line_coeff_left[4],line_coeff_left[5]};//左竖直边界方向向量
                    Eigen::Vector3d line_vector_right{line_coeff_right[3],line_coeff_right[4],line_coeff_right[5]};//右竖直边界方向向量

                    Point3d point3d_left{line_point_left};//左竖直边界点对象
                    Point3d point3d_right{line_point_right};//右竖直边界点对象
                    Vector3d vector3d_left{line_vector_left};//左竖直边界方向向量对象
                    Vector3d vector3d_right{line_vector_right};//右竖直边界方向向量对象
                    Line3d line3d_left{point3d_left, vector3d_left};//左竖直边界线对象
                    Line3d line3d_right{point3d_right, vector3d_right};//右竖直边界线对象
                    LineAndPlane line3d_left_plane{line3d_left, planeXOY};//左竖直边界线和平面对象
                    LineAndPlane line3d_right_plane{line3d_right, planeXOY};//右竖直边界线和平面对象

                    Point3d point3d_intersection_left;//左竖直边界和水平面之间的交点对象
                    Point3d point3d_intersection_right;//右竖直边界和水平面之间的交点对象
                    line3d_left_plane.GetIntersectionPoint(point3d_intersection_left);//求解交点
                    line3d_right_plane.GetIntersectionPoint(point3d_intersection_right);//求解交点

                    Eigen::Vector3d direction_vector = point3d_intersection_left.point - point3d_intersection_right.point;//求解交点间方向向量
#ifdef TEST
                    std::cout << "direction_vector: " << direction_vector << std::endl;
#endif //TEST
                    direction_vector /= direction_vector.norm();//方向向量归一化
                    if(direction_vector[1] < 0)//将方向向量变换到与水平坐标系Y轴正方向同向
                        direction_vector = -direction_vector;                    

                    for(int i = -2; i < 3; i++)
                    { 
                        //提取靠尺点云
                        float binary_right = y_limit_m + 2 * i * 50 - direction_vector(1) * 25;//靠尺点云右边界
                        float binary_left = y_limit_m + 2 * i * 50 + direction_vector(1) * 25;//靠尺点云左边界
                        std::cout << "binary_left - binary_right: " << binary_left << "-" << binary_right << "=" << binary_left - binary_right << std::endl;

                        pcl::PointCloud<pcl::PointXYZI>::Ptr ruler_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云
                        cp.ConditionRemoval(cloud_optimal, ruler_point_cloud, am::AXIS_Y, binary_right, binary_left);//提取靠尺点云
                        if(ruler_point_cloud -> size() < 100)
                        {
                            result.push_back(-1);
                            continue;
                        }
                        pcl::ModelCoefficients ruler_coeff = cp.Plane(ruler_point_cloud);//靠尺点云平面参数
                        Eigen::Vector3d ruler_normal_vector_in_world{ruler_coeff.values[0],ruler_coeff.values[1],ruler_coeff.values[2]};//靠尺点云平面法矢
                        *rulers+=*ruler_point_cloud;

                        //计算垂直度
                        double angle_radian = std::fabs(std::acos(Eigen::Vector3d::UnitZ().dot(ruler_normal_vector_in_world)) - M_PI / 2.);//计算靠尺点云平面法矢与水平坐标系Z轴之间的夹角
                        double angle_degree = std::fabs(Degree(angle_radian));//将夹角转换为角度，并求与90度之间的偏差

                        std::sort(ruler_point_cloud->points.begin(), ruler_point_cloud->points.end(), SortPointCloudZL);
                        double ruler_distance = std::fabs( (*ruler_point_cloud->points.begin()).z - (*ruler_point_cloud->points.rbegin()).z );

                        auto one_result = ruler_distance * std::tan(angle_radian) /2.0;
#ifdef TEST
                        std::cout << "angle_degree: " << angle_degree << std::endl;
                        std::cout << "ruler_distance: " << ruler_distance  << " extraParam: " << extraParam << std::endl;
                        std::cout << "one_result: " << one_result << std::endl;
#endif //TEST
                        result.push_back(one_result);
                    }
                    break;                    
                }
                default:
                {
                    std::string error = "Error in Calculate(): type is invalid.";
                    std::string statusNumber = "-1";
                    _node->_PubStatus(GenerateStatusMessage(_node->get_name(), statusNumber, error));

                    throw std::runtime_error(error);  
                }                    
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
            std::string time = GetLocalTime();
            std::string point_cloud_origin_name = "/home/ubuntu/tmp/perpendicularity/pointcloud_origin_" + time + ".txt";
            std::string point_cloud_transform_name = "/home/ubuntu/tmp/perpendicularity/pointcloud_transform_" + time + ".txt";
            std::string point_cloud_optimal_namw = "/home/ubuntu/tmp/perpendicularity/pointcloud_optimal_" + time + ".txt";
            std::string point_cloud_ruler_name = "/home/ubuntu/tmp/perpendicularity/pointcloud_ruler_" + time + ".txt";

            fo.SetProperty(point_cloud_origin_name);
            fo.Write(cloud_voxel_grid_filter);

            fo.SetProperty(point_cloud_transform_name);
            fo.Write(cloud_transform);          

            fo.SetProperty(point_cloud_optimal_namw);
            fo.Write(cloud_optimal);

            fo.SetProperty(point_cloud_ruler_name);
            fo.Write(rulers);
#endif //TEST

            return {result};
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO(_node->get_logger(),"%s Exception in Calculate(): %s.", GetLocalTime(), e.what());
            return {-1, -1, -1, -1, -1};
        }
        catch(...)
        {
            RCLCPP_INFO(_node->get_logger(),"%s Exception in Calculate(): unknown", GetLocalTime());
            return {-1, -1, -1, -1, -1};
        }   

    private:
        int status;
        double extraParam;

        Eigen::Matrix3d I_R_C;

        std::string fileName;

        Perpendicularity* _node;
};

/**** Perpendicularity ****/
Perpendicularity::Perpendicularity(const rclcpp::NodeOptions& option  = rclcpp::NodeOptions()) : rclcpp::Node("perpendicularity_and_levelness_node", option)
{
    _initThread = std::thread(&Perpendicularity::_Init, this);
}

Perpendicularity::~Perpendicularity() try
{
    _initThread.join();

    _subPointCloud.reset();
    _subRpy.reset();
    _pubResult.reset();
    _pubStatus.reset();

    RCLCPP_INFO(this->get_logger(),"%s Perpendicularity destroyed successfully.", GetLocalTime());
}
catch(const std::exception& e)
{
    RCLCPP_INFO(this->get_logger(), "%s Exception in Perpendicularity destruction: %s", GetLocalTime(), e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_INFO(this->get_logger(),"%s Exception in Perpendicularity destruction: unknown", GetLocalTime());
    rclcpp::shutdown();
}

void Perpendicularity::_Init() try
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
        std::bind(&Perpendicularity::_SubPointCloud, this, std::placeholders::_1));

    _subRpy = this->create_subscription<shared_interfaces::msg::RollPitchYaw>(
        _subRpyName, 
        10, 
        std::bind(&Perpendicularity::_SubRpy, this, std::placeholders::_1)
    );

    _pubResult = this->create_publisher<shared_interfaces::msg::Float64Array>(_pubResultName, 1);

    _pubStatus = this->create_publisher<std_msgs::msg::String>(_pubStatusName, 1);

    _status = 0;

    RCLCPP_INFO(this->get_logger(), "%s Perpendicularity initialized successfully.", GetLocalTime());
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(this->get_logger(), "%s Exception in Perpendicularity initializer: %s", GetLocalTime(), e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(this->get_logger(),"%s Exception in Perpendicularity initializer: unknown", GetLocalTime());
    rclcpp::shutdown();
}

void Perpendicularity::_InitializeParameters()
{
    return;
}

void Perpendicularity::_UpdateParameters()
{
    return;
}

void Perpendicularity::_SubPointCloud(shared_interfaces::msg::PointCloudC::UniquePtr pointCloudCPtr)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "initialize and subscribe a poing cloud.");
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

void Perpendicularity::_SubRpy(shared_interfaces::msg::RollPitchYaw::UniquePtr ptr)
{
    if(_status < 0)
        return;
    
    _roll = ptr->roll;
    _pitch = ptr->pitch;
    _yaw = ptr->yaw;
}

void Perpendicularity::_PubResult(const std::vector<double>& result)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"initialize and publish a result.");
        return;
    }

    auto msg = std::make_unique<shared_interfaces::msg::Float64Array>();
    msg -> data = result;
    _pubResult->publish(std::move(msg));

    RCLCPP_INFO(this->get_logger(), "%s publish result: %f.", GetLocalTime(), result[0]);
}

void Perpendicularity::_PubStatus(const std::string& status)
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
RCLCPP_COMPONENTS_REGISTER_NODE(am::Perpendicularity)