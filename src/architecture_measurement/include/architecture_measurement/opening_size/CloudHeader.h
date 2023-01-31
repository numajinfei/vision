#ifndef _CLOUD_HEADER_H
#define _CLOUD_HEADER_H

#include "Utils.h"
#include <thread>

//enum SortMethod 
//{ 
//    SIZE = 0,
//    NORMAL_VECTOR = 1, 
//    MASS_CENTER = 2 
//};

class CloudHeader
{
private:
    pcl::PointCloud<PointT>::Ptr cloud;
    int index;
    std::size_t size;
    Eigen::Vector3d normalVector;
    Eigen::Vector4d planeCoeff;
    Eigen::Vector3d massCenter;

private:
    void Plane(double distanceThreshold);
    void CalculateOBB();

public:
    CloudHeader();
    CloudHeader(const pcl::PointCloud<PointT>::Ptr& _cloud, const int& _index);
    CloudHeader(const CloudHeader& cloud_header);
    CloudHeader& operator=(const CloudHeader& cloud_header);
    ~CloudHeader();

    // void SetProperty(const pcl::PointCloud<PointT>::Ptr& _cloud, const int& _index);

    inline pcl::PointCloud<PointT>::Ptr GetPointCloudPtr()
    {
        return cloud;
    }

    inline int GetIndex()
    {
        return index;
    }

    inline long int GetSize()
    {
        return size;
    }

    inline Eigen::Vector3d GetNormalVector()
    {
        return normalVector;
    }

    inline Eigen::Vector4d GetPlaneCoeff()
    {
        return planeCoeff;
    }

    inline Eigen::Vector3d GetMassCenter()
    {
        return massCenter;
    }
};

using CloudHeaderVector = std::vector<CloudHeader>;

bool SortSize(CloudHeader& ch1, CloudHeader& ch2);

bool SortNormalVectorL(CloudHeader& ch1, CloudHeader& ch2);

bool SortNormalVectorG(CloudHeader& ch1, CloudHeader& ch2);

bool SortMassCenterXL(CloudHeader& ch1, CloudHeader& ch2);

bool SortMassCenterXG(CloudHeader& ch1, CloudHeader& ch2);

bool SortMassCenterYL(CloudHeader& ch1, CloudHeader& ch2);

bool SortMassCenterYG(CloudHeader& ch1, CloudHeader& ch2);

bool SortMassCenterZL(CloudHeader& ch1, CloudHeader& ch2);

bool SortMassCenterZG(CloudHeader& ch1, CloudHeader& ch2);



#endif //_CLOUD_HEADER_H