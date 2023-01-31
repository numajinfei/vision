#include "architecture_measurement/impl/cloud_pretreatment.hpp"

#include <iostream>
#include <exception>
#include <chrono>
#include <fstream>
#include <thread>

#include "pcl/features/normal_3d.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/features/moment_of_inertia_estimation.h"
#include "pcl/features/boundary.h"

#include "pcl/segmentation/region_growing.h"
#include "pcl/segmentation/sac_segmentation.h"

#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/filters/shadowpoints.h"
#include "pcl/filters/plane_clipper3D.h"

#include "pcl/sample_consensus/sac_model_plane.h"
#include "pcl/sample_consensus/sac_model_parallel_line.h"

#include "pcl/surface/concave_hull.h"

#include "pcl/visualization/pcl_visualizer.h"

#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/common/transforms.h"

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

using namespace std::chrono_literals;

namespace am
{

CloudPretreatment::CloudPretreatment() 
{    
    _paramsConfig = std::make_shared<ParamsConfig>("/home/ubuntu/am_v2_ws/install/architecture_measurement/share/architecture_measurement/config/cloud_pretreatment_params.yaml");
}

CloudPretreatment::CloudPretreatment(const CloudPretreatment& cloud_pretreatment) 
{
    _paramsConfig = cloud_pretreatment._paramsConfig;
}

CloudPretreatment::~CloudPretreatment()
{
    _paramsConfig.reset();
}

/**** common ****/
void CloudPretreatment::PointcloudTransform(pcl::PointCloud<pcl::PointXYZI>& src, pcl::PointCloud<pcl::PointXYZI>& dst, float theta)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    pcl::PointXYZI minPt, maxPt;
	pcl::getMinMax3D(src, minPt, maxPt);

    float offset_x = (minPt.x + maxPt.x) / 2;
    float offset_y = (minPt.y + maxPt.y) / 2;
    float offset_z = (minPt.z + maxPt.z) / 2;

    // std::cout << "offset_z: " << offset_z << std::endl;

    //平移
    transform.translation() << -offset_x, -offset_y , -offset_z ;
    // std::cout << "translation: " << transform.matrix() << std::endl;
    // std::this_thread::sleep_for(100ms);
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
  
    pcl::transformPointCloud(src, dst, transform);

    // std::cout << "transform " << std::endl;

    return;
}

int CloudPretreatment::PointCloudTransformation(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, 
    const Eigen::Matrix3f& rotation, 
    const Eigen::Vector3f& translation)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    transform.translation() = translation;
    transform.rotate(rotation);
  
    pcl::transformPointCloud(*srcPointCloudPtr, *dstPointCloudPtr, transform);

    return 0;
}

int CloudPretreatment::GetMinMax3D(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointXYZI& minPoint,
    pcl::PointXYZI& maxPoint)
{
    pcl::getMinMax3D(*srcPointCloudPtr, minPoint, maxPoint);
    return 0;
}

/**** filter ****/

void CloudPretreatment::StatisticalFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& res)
{
    float meanK = _paramsConfig->GetStatisticalFilter_meanK();
    float stddevMulThresh = _paramsConfig->GetStatisticalFilter_stddevMulThresh();

    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(meanK);//todo
    sor.setStddevMulThresh(stddevMulThresh);//todo
    sor.filter(*res);
}

void CloudPretreatment::RadiusFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& res)
{
    float radiusSearch = _paramsConfig->GetRadiusFilter_radius();
    int minNeighborsInRadius = _paramsConfig->GetRadiusFilter_minNeighborsInRadius();

    pcl::RadiusOutlierRemoval<pcl::PointXYZI> rad;
    rad.setInputCloud(cloud);
    rad.setRadiusSearch(radiusSearch);//todo
    rad.setMinNeighborsInRadius(minNeighborsInRadius);//todo
    rad.filter(*res);
}

void CloudPretreatment::ConditionRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& res, const int& axis, const float& lower_limit, const float& upper_limit)
{
    pcl::ConditionAnd<pcl::PointXYZI>::Ptr cond(new pcl::ConditionAnd<pcl::PointXYZI>());
    switch(axis)
    {
        case AXIS_X:
        {
            //std::cout << "AXIS_X" << std::endl;
            cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::GT,lower_limit)));
            cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::LT,upper_limit)));
            break;
        }
        case AXIS_Y:
        {
            //std::cout << "AXIS_Y" << std::endl;
            cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::GT,lower_limit)));
            cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::LT,upper_limit)));
            break;
        }
        case AXIS_Z:
        {
            //std::cout << "AXIS_Z" << std::endl;
            cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("z",pcl::ComparisonOps::GT,lower_limit)));
            cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("z",pcl::ComparisonOps::LT,upper_limit)));
            break;
        }
    }

    pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
    condrem.setInputCloud(cloud);
    condrem.setCondition(cond);
    condrem.setKeepOrganized(false);
    condrem.filter(*res);
    //std::cout << "res size: " << res -> points.size() << std::endl;
    return;
}

void CloudPretreatment::ConditionRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& res, const int& quadrant)
{
    pcl::ConditionAnd<pcl::PointXYZI>::Ptr cond(new pcl::ConditionAnd<pcl::PointXYZI>());
    switch(quadrant)
    {
        case FIRST_QUADRANT:
        {
            //std::cout << "FIRST_QUADRANT" << std::endl;
            cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::GT,0.0)));
            cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::GT,0.0)));
            break;
        }
        case SECOND_QUADRANT:
        {
            //std::cout << "SECOND_QUADRANT" << std::endl;
            cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::LT,0.0)));
            cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::GT,0.0)));
            break;
        }
        case THIRD_QUADRANT:
        {
            //std::cout << "THIRD_QUADRANT" << std::endl;
            cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::LT,0.0)));
            cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::LT,0.0)));
            break;
        }
        case FORTH_QUADRANT:
        {
            //std::cout << "FORTH_QUADRANT" << std::endl;
            cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::GT,0.0)));
            cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::LT,0.0)));
            break;
        }
        default:
        {
            throw std::runtime_error("Error in Function ConditionRemoval(): quadrant is invalid!");
        }
    }
    pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
    condrem.setInputCloud(cloud);
    condrem.setCondition(cond);
    condrem.setKeepOrganized(false);
    condrem.filter(*res);
    //std::cout << "res size: " << res -> points.size() << std::endl;
}

int CloudPretreatment::VoxelGridFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& res)
{
    float leafSizeX = _paramsConfig->GetVoxelGridFilter_leafSize_x();
    float leafSizeY = _paramsConfig->GetVoxelGridFilter_leafSize_y();
    float leafSizeZ = _paramsConfig->GetVoxelGridFilter_leafSize_z();

    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leafSizeX, leafSizeY, leafSizeZ);//todo
    vg.filter(*res);

    if(res->points.size() == 0)
    {
        return -1;
    }
    return 0;
}

int CloudPretreatment::VoxelGridFilterMultiThreads(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& res)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_first_quadrant(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_second_quadrant(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_third_quadrant(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_forth_quadrant(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_grid_first_quadrant(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_grid_second_quadrant(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_grid_third_quadrant(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_grid_forth_quadrant(new pcl::PointCloud<pcl::PointXYZI>);


    // std::vector<std::thread> threads;
    ConditionRemoval(cloud,cloud_first_quadrant,FIRST_QUADRANT);
    VoxelGridFilter(cloud_first_quadrant,cloud_grid_first_quadrant);

    ConditionRemoval(cloud,cloud_second_quadrant,SECOND_QUADRANT);
    VoxelGridFilter(cloud_second_quadrant,cloud_grid_second_quadrant);

    ConditionRemoval(cloud,cloud_third_quadrant,THIRD_QUADRANT);
    VoxelGridFilter(cloud_third_quadrant,cloud_grid_third_quadrant);

    ConditionRemoval(cloud,cloud_forth_quadrant,FORTH_QUADRANT);
    VoxelGridFilter(cloud_forth_quadrant,cloud_grid_forth_quadrant);

    *res += *cloud_grid_first_quadrant;    
    *res += *cloud_grid_second_quadrant;    
    *res += *cloud_grid_third_quadrant;    
    *res += *cloud_grid_forth_quadrant;

    //std::cout << "cloud_first_quadrant size: " << cloud_first_quadrant->size() << "    " << "cloud_grid_first_quadrant size: " << cloud_grid_first_quadrant->size() << std::endl;
    //std::cout << "cloud_second_quadrant size: " << cloud_second_quadrant->size() << "    " << "cloud_grid_second_quadrant size: " << cloud_grid_second_quadrant->size() << std::endl;
    //std::cout << "cloud_third_quadrant size: " << cloud_third_quadrant->size() << "    " << "cloud_grid_third_quadrant size: " << cloud_grid_third_quadrant->size() << std::endl;
    //std::cout << "cloud_forth_quadrant size: " << cloud_forth_quadrant->size() << "    " << "cloud_grid_forth_quadrant size: " << cloud_grid_forth_quadrant->size() << std::endl;

    if(cloud_first_quadrant->size() == 0 || cloud_second_quadrant->size() == 0 || cloud_third_quadrant->size() == 0 || cloud_forth_quadrant->size() == 0)
    {
        return -1;
    }

    return 0;
}

int CloudPretreatment::DuplicateRemoval(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr dstPointCloudPtr)
{
    pcl::search::KdTree<pcl::PointXYZI> tree;
    tree.setInputCloud(srcPointCloudPtr);

    pcl::Indices indices;
    std::vector<float> distance;
    float radius = 0.01;

    std::set<int> removalIndices;

    for(auto& point : *srcPointCloudPtr)
    {
        if(tree.radiusSearch(point, radius, indices, distance) > 0)
        {
            if(indices.size() != 1)
            {
                for(std::size_t i = 1; i < indices.size(); ++i)
                {
                    removalIndices.insert(indices[i]);
                }
            }
        }
    }

    pcl::PointIndices::Ptr outliners(new pcl::PointIndices);
    std::copy(removalIndices.cbegin(), removalIndices.cend(), std::back_inserter(outliners->indices));

    Extract(srcPointCloudPtr, outliners, dstPointCloudPtr, false, false);

    return 0;
}


void CloudPretreatment::ProjectPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& dst, const pcl::ModelCoefficients::Ptr& coeffs)
{
    pcl::ProjectInliers<pcl::PointXYZI> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coeffs);
    proj.filter (*dst);
}

void CloudPretreatment::ShadowPointsFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& normals, pcl::PointCloud<pcl::PointXYZI>::Ptr& res)
{
    float threshold = _paramsConfig->GetShadowPointsFilter_threshold();

    pcl::ShadowPoints<pcl::PointXYZI, pcl::Normal> spfilter(true);
    spfilter.setInputCloud(cloud);
    spfilter.setThreshold(threshold);
    spfilter.setNormals(normals);
    spfilter.filter(*res);
}

void CloudPretreatment::MedianFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    int k = 9;// _paramsConfig->GetMedianFilter_k();

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);

    for(std::size_t i = 0; i < cloud->size(); i++)
    {
        std::vector<int> pointIdxKNNSearch(k);
        std::vector<float> pointKNNSquaredDistance(k);

        if(kdtree.nearestKSearch(cloud->points[i], k, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
        {
            std::vector<float> intensity(k);
            for(int i = 0; i < k ; i++)
               intensity[i]=(*cloud)[pointIdxKNNSearch[i]].intensity;
            std::sort(intensity.begin(), intensity.end());
            float median_intensity = intensity[k/2];
            for(int i = 0; i < k ; i++)
               (*cloud)[pointIdxKNNSearch[i]].intensity = median_intensity;
        }
    }
}

void CloudPretreatment::PlaneClipper3D(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr& dst,      
    const Eigen::Vector4f& plane1, 
    const Eigen::Vector4f& plane2,
    const bool& nagative1,            
    const bool& nagative2)
{    
    pcl::PlaneClipper3D<pcl::PointXYZI> clipper(plane1);
    pcl::PointIndices::Ptr indicesPtr(new pcl::PointIndices);
    clipper.clipPointCloud3D(*cloud, indicesPtr->indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);
    Extract(cloud, indicesPtr, cloud_tmp, nagative1);
    clipper.setPlaneParameters(plane2);

    pcl::PointIndices::Ptr indicesPtr_dst(new pcl::PointIndices);
    clipper.clipPointCloud3D(*cloud_tmp, indicesPtr_dst->indices);
    Extract(cloud, indicesPtr_dst, dst, nagative2);

    return;
}

void CloudPretreatment::PlaneClipper3D(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr& dst,      
    const Eigen::Vector4f& plane,
    const bool& nagative)
{  
    pcl::PlaneClipper3D<pcl::PointXYZI> clipper(plane);
    pcl::PointIndices::Ptr indicesPtr(new pcl::PointIndices);
    clipper.clipPointCloud3D(*cloud, indicesPtr->indices);
    Extract(cloud, indicesPtr, dst, nagative);

    return;
}


/**** features ****/

void CloudPretreatment::NormalEstimation(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,pcl::PointCloud<pcl::Normal>::Ptr& normal)
{
    int numberOfThreads = _paramsConfig->GetNormalEstimation_numberOfThreads();
    int kSearch = _paramsConfig->GetNormalEstimation_kSearch();

    pcl::search::Search<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne_omp;
    ne_omp.setNumberOfThreads(numberOfThreads);//todo
    ne_omp.setSearchMethod(tree);
    ne_omp.setInputCloud(cloud);
    ne_omp.setKSearch(kSearch);//todo
    ne_omp.compute(*normal);
}

void CloudPretreatment::OrientedBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                                            pcl::PointXYZI& minPoint, 
                                            pcl::PointXYZI& maxPoint, 
                                            pcl::PointXYZI& position,
                                            Eigen::Matrix3f& rotationMatrix,
                                            Eigen::Vector3f& majorVector,
                                            Eigen::Vector3f& middleVector,
                                            Eigen::Vector3f& minorVector,
                                            Eigen::Vector3f& massCenter)
{
    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    feature_extractor.getOBB(minPoint, maxPoint, position, rotationMatrix); 
    // feature_extractor.getEigenValues(major_value,middle_value, minor_value); 
    feature_extractor.getEigenVectors(majorVector,middleVector, minorVector); 
    feature_extractor.getMassCenter(massCenter);
}

void CloudPretreatment::AxisAlignedBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                                                pcl::PointXYZI& minPoint, 
                                                pcl::PointXYZI& maxPoint, 
                                                Eigen::Vector3f& majorVector,
                                                Eigen::Vector3f& middleVector,
                                                Eigen::Vector3f& minorVector,
                                                Eigen::Vector3f& massCenter)
{
    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    feature_extractor.getAABB(minPoint, maxPoint); 
    // feature_extractor.getEigenValues(major_value,middle_value, minor_value); 
    feature_extractor.getEigenVectors(majorVector,middleVector, minorVector); 
    feature_extractor.getMassCenter(massCenter);
}

void CloudPretreatment::Boundary(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& normal, pcl::PointCloud<pcl::PointXYZI>::Ptr& res)
{
    float radius = _paramsConfig->GetBoundry_radius();
    float angleThreshold = _paramsConfig->GetBoundry_angleThreshold();
    angleThreshold = M_PI / angleThreshold;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZI, pcl::Normal, pcl::Boundary> be;
    be.setInputCloud(cloud);
    be.setInputNormals(normal);
    be.setRadiusSearch(radius);
    be.setAngleThreshold(angleThreshold);
    be.setSearchMethod(tree);

    be.compute(boundaries);

    for(size_t i = 0; i < cloud->size(); i++)
    {
        if(boundaries[i].boundary_point > 0)
        {
            (*res).push_back(cloud->points[i]);
        }        
    }
}

/**** segmentation ****/

void CloudPretreatment::Extract(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::PointIndices::Ptr& indicesPtr, pcl::PointCloud<pcl::PointXYZI>::Ptr& dst,const bool& negative)
{
    pcl::ExtractIndices<pcl::PointXYZI> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(indicesPtr);
    ei.setNegative (negative);
    ei.filter(*dst);
}

void CloudPretreatment::Extract(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::PointIndices::Ptr& indicesPtr, pcl::PointCloud<pcl::PointXYZI>::Ptr& dst,const bool& negative, const bool& keepOrginized)
{
    pcl::ExtractIndices<pcl::PointXYZI> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(indicesPtr);
    ei.setNegative (negative);
    ei.setKeepOrganized (keepOrginized);
    ei.filter(*dst);
}

pcl::ModelCoefficients CloudPretreatment::Plane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    return Plane(cloud, inliers);
}

pcl::ModelCoefficients CloudPretreatment::Plane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers)
{
    float distanceThreshold = _paramsConfig->GetPlane_distanceThreshold();

    pcl::ModelCoefficients coefficients;
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);//todo
    seg.setInputCloud(cloud);
    seg.segment(*inliers, coefficients);

    return coefficients;
}

int CloudPretreatment::ParallelLine(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Eigen::Vector3f& axis ,const int& number)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    return ParallelLine(cloud, inliers, axis, number);
}

int CloudPretreatment::ParallelLine(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointIndices::Ptr& inliers, const Eigen::Vector3f& axis,const int& number)
{
    try
    {
        float epsAngle = _paramsConfig->GetParallelLine_epsAngle();
        float distanceThreshold = _paramsConfig->GetParallelLine_distanceThreshold();

        pcl::ModelCoefficients coefficients;
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PARALLEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setEpsAngle(epsAngle); //todo
        seg.setAxis(axis);
        seg.setDistanceThreshold(distanceThreshold); //todo
        seg.setInputCloud(cloud);
        seg.segment(*inliers, coefficients);
        //std::cout << "inliers size: " <<inliers->indices.size() << std::endl;
        if(inliers->indices.size() < number)
            return -1;
        return 0;
    }
    catch(const std::exception& e)
    {
        return -1;
    } 

}

pcl::ModelCoefficients CloudPretreatment::Line(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointIndices::Ptr& inliers)
{
    float distanceThreshold = _paramsConfig->GetParallelLine_distanceThreshold();

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);   
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    return *coefficients;
}

void CloudPretreatment::PalneSegmentation(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& normal, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cloudVector)
{
    int minClusterSize = _paramsConfig->GetPalneSegmentation_minClusterSize();
    int MaxClusterSize = _paramsConfig->GetPalneSegmentation_maxClusterSize();
    int numberOfNeighbours = _paramsConfig->GetPalneSegmentation_numberOfNeighbours();
    float smoothnessThreshold = _paramsConfig->GetPalneSegmentation_smoothnessThreshold();
    float curvatureThreshold = _paramsConfig->GetPalneSegmentation_curvatureThreshold();

    pcl::search::Search<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> regions;

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> rg;
    rg.setMinClusterSize(minClusterSize); //todo
    rg.setMaxClusterSize(MaxClusterSize); //todo
    rg.setSearchMethod(tree);
    rg.setNumberOfNeighbours(numberOfNeighbours); //todo
    rg.setInputCloud(cloud);
    rg.setInputNormals(normal);
    rg.setSmoothnessThreshold(smoothnessThreshold); //todo
    rg.setCurvatureThreshold(curvatureThreshold); //todo
    rg.extract(regions);

    //std::cout << "regions size: " << regions.size() << std::endl;
    cloudVector.resize(regions.size());

    for(size_t i = 0; i < regions.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointIndices::Ptr indicePtr(new pcl::PointIndices(regions[i]));
        Extract(cloud, indicePtr, cloud_temp, false);
        cloudVector[i] = std::move(cloud_temp);
    }
}

/**** surface ****/
void CloudPretreatment::ConcaveHull(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& dst, const int& demension)
{
    float alpha = _paramsConfig->GetConcaveHull_alpha();

    pcl::ConcaveHull<pcl::PointXYZI> chull;
    chull.setInputCloud (cloud);
    chull.setAlpha (alpha);//todo
    chull.setDimension (demension);
    chull.reconstruct (*dst);
}

/**** visualization ****/
void Viewer_PlaneClipper3D(pcl::PointCloud<pcl::PointXYZI>::Ptr& before, pcl::PointCloud<pcl::PointXYZI>::Ptr& after)
{
    pcl::visualization::PCLVisualizer::Ptr view(new pcl::visualization::PCLVisualizer("show_point_cloud"));

    int v1(0);
    view -> createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    view -> setBackgroundColor(0, 0, 0, v1);
    view -> addText("origin point cloud", 10 , 10, "v1_test", v1);

    int v2(0);
    view -> createViewPort(0.5 ,0.0, 0.5, 1.0, v2);
    view -> setBackgroundColor(0.1, 0.1, 0.1, v2);
    view -> addText("filter point cloud", 10, 10, "v2_test", v2);

    view -> addPointCloud<pcl::PointXYZI>(before, "origin point cloud", v1);
    view -> addPointCloud<pcl::PointXYZI>(after,"filter point cloud", v2);
    view -> setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0,"origin point cloud", v1);
    view -> setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0,"filter point cloud", v2);

    view -> addCoordinateSystem(0.1);

    while(!view->wasStopped())
    {
        view->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return;
}






CloudPretreatment::ParamsConfig::ParamsConfig()
{

}

CloudPretreatment::ParamsConfig::ParamsConfig(const std::string& fileName)
{
    Load(fileName);    
    // PrintParam();
    Parse();
}

CloudPretreatment::ParamsConfig::~ParamsConfig()
{

}

void CloudPretreatment::ParamsConfig::Load(const std::string& fileName)
{
    std::ifstream file(fileName,std::ifstream::in);
    node = YAML::Load(file);
}

void CloudPretreatment::ParamsConfig::Parse()
{
    YAML::Node params_node = node["cloud_pretreatment_node"]["params"];

    YAML::Node statisticalFilter_node = params_node["statisticalFilter"];   
    statisticalFilter_meanK = statisticalFilter_node["statisticalFilter_meanK"].as<float>();
    statisticalFilter_stddevMulThresh = statisticalFilter_node["statisticalFilter_stddevMulThresh"].as<float>();

    YAML::Node radiusFilter_node = params_node["radiusFilter"];
    radiusFilter_radius = radiusFilter_node["radiusFilter_radius"].as<float>();
    radiusFilter_minNeighborsInRadius = radiusFilter_node["radiusFilter_minNeighborsInRadius"].as<int>();

    YAML::Node voxelGridFilter_node = params_node["voxelGridFilter"];
    voxelGridFilter_leafSize_x = voxelGridFilter_node["voxelGridFilter_leafSize_x"].as<float>();
    voxelGridFilter_leafSize_y = voxelGridFilter_node["voxelGridFilter_leafSize_y"].as<float>();
    voxelGridFilter_leafSize_z = voxelGridFilter_node["voxelGridFilter_leafSize_z"].as<float>();

    YAML::Node shadowPointsFilter_node = params_node["shadowPointsFilter"];
    shadowPointsFilter_threshold = shadowPointsFilter_node["shadowPointsFilter_threshold"].as<float>();

    YAML::Node normalEstimation_node = params_node["normalEstimation"];
    normalEstimation_numberOfThreads = normalEstimation_node["normalEstimation_numberOfThreads"].as<int>();
    normalEstimation_kSearch = normalEstimation_node["normalEstimation_kSearch"].as<int>();

    YAML::Node plane_node = params_node["plane"];
    plane_distanceThreshold = plane_node["plane_distanceThreshold"].as<float>();

    YAML::Node parallelLine_node = params_node["parallelLine"];
    parallelLine_epsAngle = parallelLine_node["parallelLine_epsAngle"].as<float>();
    parallelLine_distanceThreshold = parallelLine_node["parallelLine_distanceThreshold"].as<float>();

    YAML::Node boundary_node = params_node["boundary"];
    boundary_radius = boundary_node["boundary_radius"].as<float>();
    boundary_angleThreshold = boundary_node["boundary_angleThreshold"].as<float>();


    YAML::Node planeSegmentation_node = params_node["planeSegmentation"];
    planeSegmentation_minClusterSize = planeSegmentation_node["planeSegmentation_minClusterSize"].as<int>();
    planeSegmentation_maxClusterSize = planeSegmentation_node["planeSegmentation_maxClusterSize"].as<int>();
    planeSegmentation_numberOfNeighbours = planeSegmentation_node["planeSegmentation_numberOfNeighbours"].as<int>();
    planeSegmentation_smoothnessThreshold = planeSegmentation_node["planeSegmentation_smoothnessThreshold"].as<float>();
    planeSegmentation_curvatureThreshold = planeSegmentation_node["planeSegmentation_curvatureThreshold"].as<float>();

    YAML::Node concaveHull_node = params_node["concaveHull"];
    concaveHull_alpha = concaveHull_node["concaveHull_alpha"].as<float>();
}

void CloudPretreatment::ParamsConfig::PrintParam()
{
    std::cout 
        << "/**** CloudPretreatment param config ****/"
        << std::endl
        << std::endl
        << node 
        << std::endl 
        << std::endl;
}

}//namespace am