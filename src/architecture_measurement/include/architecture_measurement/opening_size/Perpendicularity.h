#ifndef PERPENDICULARITY_H
#define PERPENDICULARITY_H

#include "CloudHeader.h"
#include "SpaceAnalyticGeometry.h"

//constexpr double THRESHOLD = std::sqrt(2)/ 2.0;
constexpr double THRESHOLD = 1.4142 / 2.0;
constexpr double extraParam = 2.0;

enum PERPENDICULARITY_TYPE 
{ 
	SIDEWALL = 0,
	PILLAR = 1 
};

class VerticalBoundary
{
public:
    VerticalBoundary(pcl::ModelCoefficients _coef, pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud);
    ~VerticalBoundary();

    inline float GetX()
    {
        return x;
    }

    inline float GetY()
    {
        return y;
    }

    inline float GetZ()
    {
        return z;
    }

    inline float GetLength()
    {
        return length;
    }

private:
    void CalculateLineLength();
    void ClaculateLineCenter();
    void CalculateLimitsZ();

public:
    pcl::ModelCoefficients coef;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    float x;
    float y;
    float z;
    float length;
    float limit_z_s;
    float limit_z_l;
};


class Perpendicularity
{
public:
	Perpendicularity();
	~Perpendicularity();
    
    std::vector<double> Calculate(pcl::PointCloud<PointT>& cloud, const int& type, double& roll, double& pitch, double& yaw);

private:

    int SortCloud(std::vector<pcl::PointCloud<PointT>::Ptr>& clouds, const int& type, int& index);
    
    void SideWallPerpendicularity(Eigen::Matrix3d W_R_C, pcl::PointCloud<PointT>& cloud_optimal, pcl::PointCloud<PointT>& rulers,
        pcl::ModelCoefficients& plane_coeff, const double& distanceThreshold, std::vector<double> result);

    void PillarPerpendicularityPretreatment(pcl::PointCloud<PointT>& cloud_optimal,
        std::vector<VerticalBoundary> verticalBoundaries);

    void PillarPerpendicularity(VerticalBoundary vertical_boundary_left, VerticalBoundary vertical_boundary_right,
        pcl::PointCloud<PointT>& cloud_optimal, pcl::PointCloud<PointT>& rulers,
        float& y_limit_m, const double& distanceThreshold, std::vector<double> result);
    
};


#endif  //PERPENDICULARITY_H