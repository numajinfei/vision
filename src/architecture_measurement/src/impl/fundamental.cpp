#include "architecture_measurement/impl/fundamental.hpp"

namespace am
{

/**
 * @brief degree convert to radian
 * 
 * @param degree angle degree
 * @return Type angle radian
 */
double Radian(const double& degree)
{
    double radian = degree / 180. * M_PI;
    return radian;
}

/**
 * @brief 
 * 
 * @param degree 
 * @return float 
 */
float Radian(const float& degree)
{
    float radian = degree / 180. * M_PI;
    return radian;
}

/**
 * @brief radian convert to degree
 * 
 * @param radian angle radian
 * @return Type angle degree
 */
double Degree(const double& radian)
{
    double degree = radian / M_PI * 180;
    return degree;  
}

/**
 * @brief 
 * 
 * @param radian 
 * @return float 
 */
float Degree(const float& radian)
{
    float degree = radian / M_PI * 180;
    return degree;  
}

std::unordered_map<std::string, int> LUTEngineering()
{
    std::unordered_map<std::string, int> lookUpTable;    
    lookUpTable.emplace("Normal",0);
    lookUpTable.emplace("Concerte",1);
    lookUpTable.emplace("Masonry",2);
    lookUpTable.emplace("Plaster",3);
    lookUpTable.emplace("GroundBase",4);
    lookUpTable.emplace("GroundSurface",5);
    lookUpTable.emplace("Partition",6);
    lookUpTable.emplace("Veneer",7);

    return lookUpTable;
}

std::unordered_map<std::string, int>  LUTOption()
{
    std::unordered_map<std::string, int> lookUpTable;
    lookUpTable.emplace("RoofLevel", 2);
    lookUpTable.emplace("GroundLevel", 1);
    lookUpTable.emplace("WallVertical", 0);
    lookUpTable.emplace("TopPlateFlatness", 2);
    lookUpTable.emplace("GroundFlatness", 1);
    lookUpTable.emplace("SideWallFlatness", 0);
    lookUpTable.emplace("ExternalCorner", 5);
    lookUpTable.emplace("InsideCorner", 4);
    lookUpTable.emplace("PillarSectionSize", 3);
    lookUpTable.emplace("PillarVerticality", 3);
    lookUpTable.emplace("DefectDetection", -1);
    lookUpTable.emplace("EmbeddedParts", -1);
    lookUpTable.emplace("StoreyHeight", -1);
    lookUpTable.emplace("DoorOpeningSize", 6);
    lookUpTable.emplace("WindowOpeningSize", 7);

    return lookUpTable;
}



bool EndsWith(const std::string& value, const std::string& ending)
{
    if(value.size() < ending.size())
        return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

CombinationDate::CombinationDate() 
{
    _pointCloudID = -1;
    _imageID = -1;
}

CombinationDate::~CombinationDate()
{
    _pointCloudPtr.reset();
}

int CombinationDate::Complete()
{
    if(_pointCloudID > 0 && _imageID > 0)
    {
        if(_pointCloudID == _imageID)
        {
            return true;
        }
    }
    return false;
}

// VerticalBoundary::VerticalBoundary(
//     pcl::ModelCoefficients& coeffs, 
//     pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr)
// {
//     _coeffs = coeffs;
//     _pointCloudPtr = pointCloudPtr;

//     CalculateLineLength(); 
//     ClaculateLineCenter();   
//     CalculateLimitsZ();  
// }

// VerticalBoundary::~VerticalBoundary()
// {
//     _pointCloudPtr.reset();
// }

// void VerticalBoundary::CalculateLineLength()
// {
//     std::sort(_pointCloudPtr->begin(), _pointCloudPtr->end(), SortPointCloudZL);
//     pcl::PointXYZI pointBundary1 = *(_pointCloudPtr->begin());
//     pcl::PointXYZI pointBundary2 = *(_pointCloudPtr->end()-1);
//     Eigen::Vector3f pointBundaryVec1{pointBundary1.x, pointBundary1.y, pointBundary1.z};
//     Eigen::Vector3f pointBundaryVec2{pointBundary2.x, pointBundary2.y, pointBundary2.z};
//     Eigen::Vector3f vector = pointBundaryVec2 - pointBundaryVec1;
//     _length = vector.norm();
// }

// void VerticalBoundary::ClaculateLineCenter()
// {
//     std::sort(_pointCloudPtr->begin(), _pointCloudPtr->end(), SortPointCloudZL);
//     pcl::PointXYZI pointBundary1 = *(_pointCloudPtr->begin());
//     pcl::PointXYZI pointBundary2 = *(_pointCloudPtr->end()-1);
//     Eigen::Vector3f pointBundaryVec1{pointBundary1.x, pointBundary1.y, pointBundary1.z};
//     Eigen::Vector3f pointBundaryVec2{pointBundary2.x, pointBundary2.y, pointBundary2.z};
//     Eigen::Vector3f center = (pointBundaryVec1 + pointBundaryVec2) / 2.;
//     _x = center(0);
//     _y = center(1);
//     _z = center(2);
// }

// void VerticalBoundary::CalculateLimitsZ()
// {
//     auto points = _pointCloudPtr->points;
//     std::sort(points.begin(), points.end(), SortPointCloudZL);//将竖直边界点云按照Z坐标从小到大进行排序
//     points.erase(points.begin(), points.begin() + 3);
//     points.erase(points.end() - 3, points.end());//竖直边界两端部分点

//     _limitZS = points.begin() -> z;//左侧竖直边界Z坐标最小值
//     _limitZL = points.rbegin() -> z;//左侧竖直边界Z坐标最大值
// }


// bool SortVerticalBoundaryXL(VerticalBoundary& a, VerticalBoundary& b)
// {
//     return a._x < b._x;
// }

// bool SortVerticalBoundaryXG(VerticalBoundary& a, VerticalBoundary& b)
// {
//     return a._x > b._x;
// }

// bool SortVerticalBoundaryYL(VerticalBoundary& a, VerticalBoundary& b)
// {
//     return a._y < b._y;
// }

// bool SortVerticalBoundaryYG(VerticalBoundary& a, VerticalBoundary& b)
// {
//     return a._y > b._y;
// }

// bool SortVerticalBoundaryZL(VerticalBoundary& a, VerticalBoundary& b)
// {
//     return a._z < b._z;
// }

// bool SortVerticalBoundaryZG(VerticalBoundary& a, VerticalBoundary& b)
// {
//     return a._z > b._z;
// }

// bool SortLengthL(VerticalBoundary& a, VerticalBoundary& b)
// {
//     return a._length < b._length;
// }

// bool SortLengthG(VerticalBoundary& a, VerticalBoundary& b)
// {
//     return a._length > b._length;
// }


// //按照直线上一点X坐标到原点的距离，从小到大对直线进行排序
// bool SortIntersectionLineXC(IntersectionLine& line1, IntersectionLine& line2)
// {
//     auto point1 = line1.point;
//     auto point2 = line2.point;
//     return std::fabs(point1(0)) < std::fabs(point2(0));
// }

// //按照直线上一点Y坐标到原点的距离，从小到大对直线进行排序
// bool SortIntersectionLineYC(IntersectionLine& line1, IntersectionLine& line2)
// {
//     auto point1 = line1.point;
//     auto point2 = line2.point;
//     return std::fabs(point1(1)) < std::fabs(point2(1));
// }

// //按照直线上一点Z坐标到原点的距离，从小到大对直线进行排序
// bool SortIntersectionLineZC(IntersectionLine& line1, IntersectionLine& line2)
// {
//     auto point1 = line1.point;
//     auto point2 = line2.point;
//     return std::fabs(point1(2)) < std::fabs(point2(2));
// }

// bool SortPointCloudXL(pcl::PointXYZI& point1, pcl::PointXYZI& point2)
// {
//     return point1.x < point2.x;
// }

// bool SortPointCloudXG(pcl::PointXYZI& point1, pcl::PointXYZI& point2)
// {
//     return point1.x > point2.x;
// }

// bool SortPointCloudYL(pcl::PointXYZI& point1, pcl::PointXYZI& point2)
// {
//     return point1.y < point2.y;
// }

// bool SortPointCloudYG(pcl::PointXYZI& point1, pcl::PointXYZI& point2)
// {
//     return point1.y > point2.y;
// }

// bool SortPointCloudZL(pcl::PointXYZI& point1, pcl::PointXYZI& point2)
// {
//     return point1.z < point2.z;
// }

// bool SortPointCloudZG(pcl::PointXYZI& point1, pcl::PointXYZI& point2)
// {
//     return point1.z > point2.z;
// }

}//namespace am