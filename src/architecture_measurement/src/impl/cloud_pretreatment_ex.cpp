/**
 * @file cloud_pretreatment.cpp
 * @author Zhongliang Yin (yinzhongliang@163.com)
 * @brief point cloud pretreatment
 * @version 0.1
 * @date 2022-10-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "architecture_measurement/impl/cloud_pretreatment_ex.hpp"
// #include "architecture_measurement/impl/cloud_pretreatment.hpp"

using namespace std::chrono_literals;

namespace am
{

CloudPretreatmentEx::CloudPretreatmentEx() 
{    
    _paramsConfig = std::make_shared<ParamsConfig>("/home/ubuntu/am_v2_ws/install/architecture_measurement/share/architecture_measurement/config/cloud_pretreatment_params_ex.yaml");

}

CloudPretreatmentEx::CloudPretreatmentEx(const CloudPretreatmentEx& cloud_pretreatment) 
{
    _paramsConfig = cloud_pretreatment._paramsConfig;
}

CloudPretreatmentEx::~CloudPretreatmentEx()
{
    _paramsConfig.reset();
}

/**** common ****/

int CloudPretreatmentEx::PointCloudTransformation(
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

int CloudPretreatmentEx::GetMinMax3D(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointXYZI& minPoint,
    pcl::PointXYZI& maxPoint)
{
    pcl::getMinMax3D(*srcPointCloudPtr, minPoint, maxPoint);
    return 0;
}

int CloudPretreatmentEx::StatisticalFilter(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, 
    const int& engineering)
{
    float meanK = _paramsConfig -> statisticalFilter_meanK;
    float stddevMulThresh = _paramsConfig -> statisticalFilter_stddevMulThresh;

    switch (engineering)
    {
    case MASONRY:
        break;    
    default:
        break;
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(srcPointCloudPtr);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddevMulThresh);
    sor.filter(*dstPointCloudPtr);

    return 0;
}

int CloudPretreatmentEx::RadiusFilter(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, 
    const int& engineering)
{
    float radiusSearch = _paramsConfig -> radiusFilter_radius;
    int minNeighborsInRadius = _paramsConfig -> radiusFilter_minNeighborsInRadius;

    switch (engineering)
    {
    case MASONRY:
        break;    
    default:
        break;
    }

    pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror;
    ror.setInputCloud(srcPointCloudPtr);
    ror.setRadiusSearch(radiusSearch);
    ror.setMinNeighborsInRadius(minNeighborsInRadius);
    ror.filter(*dstPointCloudPtr);

    return 0;
}

// int CloudPretreatmentEx::ConditionRemoval(
//     pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
//     pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, 
//     const int& axis, 
//     const float& lower_limit, 
//     const float& upper_limit)
// {
//     pcl::ConditionAnd<pcl::PointXYZI>::Ptr cond(new pcl::ConditionAnd<pcl::PointXYZI>());
//     switch(axis)
//     {
//         case AXIS_X:
//         {
//             cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::GT,lower_limit)));
//             cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::LT,upper_limit)));
//             break;
//         }
//         case AXIS_Y:
//         {
//             cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::GT,lower_limit)));
//             cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::LT,upper_limit)));
//             break;
//         }
//         case AXIS_Z:
//         {
//             cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("z",pcl::ComparisonOps::GT,lower_limit)));
//             cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("z",pcl::ComparisonOps::LT,upper_limit)));
//             break;
//         }
//         default:
//         {
//             break;
//         }
//     }

//     pcl::ConditionalRemoval<pcl::PointXYZI> cr;
//     cr.setInputCloud(srcPointCloudPtr);
//     cr.setCondition(cond);
//     cr.setKeepOrganized(false); //todo
//     cr.filter(*dstPointCloudPtr);

//     return 0;
// }

// int CloudPretreatmentEx::ConditionRemoval(
//     pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
//     pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, 
//     const int& quadrant)
// {
//     pcl::ConditionAnd<pcl::PointXYZI>::Ptr cond(new pcl::ConditionAnd<pcl::PointXYZI>());
//     switch(quadrant)
//     {
//         case FIRST_QUADRANT:
//         {
//             cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::GT,0.0)));
//             cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::GT,0.0)));
//             break;
//         }
//         case SECOND_QUADRANT:
//         {
//             cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::LT,0.0)));
//             cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::GT,0.0)));
//             break;
//         }
//         case THIRD_QUADRANT:
//         {
//             cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::LT,0.0)));
//             cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::LT,0.0)));
//             break;
//         }
//         case FORTH_QUADRANT:
//         {
//             cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::GT,0.0)));
//             cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::LT,0.0)));
//             break;
//         }
//         default:
//         {
//             break;
//         }
//     }

//     pcl::ConditionalRemoval<pcl::PointXYZI> cr;
//     cr.setInputCloud(srcPointCloudPtr);
//     cr.setCondition(cond);
//     cr.setKeepOrganized(false);//todo
//     cr.filter(*dstPointCloudPtr);

//     return 0;
// }

int CloudPretreatmentEx::VoxelGridFilter(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, 
    const int& engineering)
{
    float leafSizeX = _paramsConfig->voxelGridFilter_leafSize_x;
    float leafSizeY = _paramsConfig->voxelGridFilter_leafSize_y;
    float leafSizeZ = _paramsConfig->voxelGridFilter_leafSize_z;

    switch (engineering)
    {
    case MASONRY:
        break;    
    default:
        break;
    }

    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(srcPointCloudPtr);
    vg.setLeafSize(leafSizeX, leafSizeY, leafSizeZ);//todo
    vg.filter(*dstPointCloudPtr);

    return 0;
}

int CloudPretreatmentEx::DuplicateRemoval(
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

int CloudPretreatmentEx::ProjectPoints(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, 
    pcl::ModelCoefficients::Ptr& coeffs)
{
    pcl::ProjectInliers<pcl::PointXYZI> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (srcPointCloudPtr);
    proj.setModelCoefficients (coeffs);
    proj.filter (*dstPointCloudPtr);

    return 0;
}

int CloudPretreatmentEx::ShadowPointsFilter(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointCloud<pcl::Normal>::Ptr& pointCloudNormalPtr, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr)
{
    float threshold = _paramsConfig->shadowPointsFilter_threshold;

    pcl::ShadowPoints<pcl::PointXYZI, pcl::Normal> sp(true);
    sp.setInputCloud(srcPointCloudPtr);
    sp.setThreshold(threshold);
    sp.setNormals(pointCloudNormalPtr);
    sp.filter(*dstPointCloudPtr);

    return 0;
}

// void CloudPretreatmentEx::MedianFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
// {
//     int k = 9;// _paramsConfig->GetMedianFilter_k();

//     pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
//     kdtree.setInputCloud(cloud);

//     for(std::size_t i = 0; i < cloud->size(); i++)
//     {
//         std::vector<int> pointIdxKNNSearch(k);
//         std::vector<float> pointKNNSquaredDistance(k);

//         if(kdtree.nearestKSearch(cloud->points[i], k, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
//         {
//             std::vector<float> intensity(k);
//             for(int i = 0; i < k ; i++)
//                intensity[i]=(*cloud)[pointIdxKNNSearch[i]].intensity;
//             std::sort(intensity.begin(), intensity.end());
//             float median_intensity = intensity[k/2];
//             for(int i = 0; i < k ; i++)
//                (*cloud)[pointIdxKNNSearch[i]].intensity = median_intensity;
//         }
//     }
// }

int CloudPretreatmentEx::PlaneClipper3D(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr,      
    const Eigen::Vector4f& plane1, 
    const Eigen::Vector4f& plane2,
    const bool& nagative1,            
    const bool& nagative2)
{    
    // pcl::PlaneClipper3D<pcl::PointXYZI> pc(plane1);

    // pcl::PointIndices::Ptr indicesPtr1(new pcl::PointIndices);
    // pcl::PointIndices::Ptr indicesPtr2(new pcl::PointIndices);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr tmpPointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);

    // pc.clipPointCloud3D(*srcPointCloudPtr, indicesPtr1->indices);    
    // Extract(srcPointCloudPtr, indicesPtr1, tmpPointCloudPtr, nagative1);

    // pc.setPlaneParameters(plane2);  
    // pc.clipPointCloud3D(*tmpPointCloudPtr, indicesPtr2->indices);
    // Extract(srcPointCloudPtr, indicesPtr2, dstPointCloudPtr, nagative2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpPointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
    PlaneClipper3D(srcPointCloudPtr, tmpPointCloudPtr, plane1, nagative1);
    PlaneClipper3D(tmpPointCloudPtr, dstPointCloudPtr, plane2, nagative2);

    return 0;
}

int CloudPretreatmentEx::PlaneClipper3D(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr,      
    const Eigen::Vector4f& plane,
    const bool& nagative)
{  
    pcl::PlaneClipper3D<pcl::PointXYZI> cp(plane);
    pcl::PointIndices::Ptr indicesPtr(new pcl::PointIndices);
    cp.clipPointCloud3D(*srcPointCloudPtr, indicesPtr->indices);
    Extract(srcPointCloudPtr, indicesPtr, dstPointCloudPtr, nagative);

    return 0;
}

int CloudPretreatmentEx::NormalEstimation(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointCloud<pcl::Normal>::Ptr& pointCloudNromalPtr, 
    const int& engineering)
{
    int numberOfThreads = _paramsConfig->normalEstimation_numberOfThreads;
    int kSearch = _paramsConfig->normalEstimation_kSearch;

    switch (engineering)
    {
    case MASONRY:
        numberOfThreads = _paramsConfig -> masonryWall.normalEstimation_numberOfThreads;
        kSearch = _paramsConfig -> masonryWall.normalEstimation_kSearch;
        break;    
    default:
        break;
    }

    pcl::search::Search<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne;
    ne.setNumberOfThreads(numberOfThreads);//todo
    ne.setSearchMethod(tree);
    ne.setInputCloud(srcPointCloudPtr);
    ne.setKSearch(kSearch);//todo
    ne.compute(*pointCloudNromalPtr);

    return 0;
}

int CloudPretreatmentEx::OrientedBoundingBox(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointXYZI& minPoint, 
    pcl::PointXYZI& maxPoint, 
    pcl::PointXYZI& position,
    Eigen::Matrix3f& rotationMatrix,
    Eigen::Vector3f& majorVector,
    Eigen::Vector3f& middleVector,
    Eigen::Vector3f& minorVector,
    Eigen::Vector3f& massCenter)
{
    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> moie;
    moie.setInputCloud(srcPointCloudPtr);
    moie.compute();

    moie.getOBB(minPoint, maxPoint, position, rotationMatrix); 
    // moie.getEigenValues(major_value,middle_value, minor_value); 
    moie.getEigenVectors(majorVector,middleVector, minorVector); 
    moie.getMassCenter(massCenter);

    return 0;
}

int CloudPretreatmentEx::AxisAlignedBoundingBox(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointXYZI& minPoint, 
    pcl::PointXYZI& maxPoint, 
    Eigen::Vector3f& majorVector,
    Eigen::Vector3f& middleVector,
    Eigen::Vector3f& minorVector,
    Eigen::Vector3f& massCenter)
{
    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> moie;
    moie.setInputCloud(srcPointCloudPtr);
    moie.compute();

    moie.getAABB(minPoint, maxPoint); 
    // moie.getEigenValues(major_value,middle_value, minor_value); 
    moie.getEigenVectors(majorVector,middleVector, minorVector); 
    moie.getMassCenter(massCenter);

    return 0;
}

int CloudPretreatmentEx::Boundary(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointCloud<pcl::Normal>::Ptr& pointCloudNormalPtr, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr)
{
    dstPointCloudPtr -> points.reserve(srcPointCloudPtr -> points.size());

    float radius = _paramsConfig->boundary_radius;
    float angleThreshold = _paramsConfig->boundary_angleThreshold;
    angleThreshold = M_PI / angleThreshold;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud<pcl::Boundary> pointCloudBoundary;
    pcl::BoundaryEstimation<pcl::PointXYZI, pcl::Normal, pcl::Boundary> be;
    be.setInputCloud(srcPointCloudPtr);
    be.setInputNormals(pointCloudNormalPtr);
    be.setRadiusSearch(radius);
    be.setAngleThreshold(angleThreshold);
    be.setSearchMethod(tree);
    be.compute(pointCloudBoundary);

    for(size_t i = 0; i < srcPointCloudPtr->size(); i++)
    {
        if(pointCloudBoundary[i].boundary_point > 0)
        {
            (*dstPointCloudPtr).push_back(srcPointCloudPtr->points[i]);
        }        
    }

    return 0;
}

int CloudPretreatmentEx::Extract(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr,
    pcl::PointIndices::Ptr& indicesPtr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr,
    const bool& negative,
    const bool& keepOrginized)
{
    pcl::ExtractIndices<pcl::PointXYZI> ei;
    ei.setInputCloud(srcPointCloudPtr);
    ei.setIndices(indicesPtr);
    ei.setNegative (negative);
    ei.setKeepOrganized (keepOrginized);
    ei.filter(*dstPointCloudPtr);

    return 0;
}

pcl::ModelCoefficients CloudPretreatmentEx::Plane(pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr)
{
    pcl::PointIndices::Ptr inliersPtr(new pcl::PointIndices);
    return Plane(srcPointCloudPtr, inliersPtr);
}

pcl::ModelCoefficients CloudPretreatmentEx::Plane(pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, pcl::PointIndices::Ptr& inliersPtr)
{
    float distanceThreshold = _paramsConfig->plane_distanceThreshold;
    std::cout << "distanceThreshold: " << distanceThreshold << std::endl;

    pcl::ModelCoefficients coefficients;
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(false);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(srcPointCloudPtr);
    seg.segment(*inliersPtr, coefficients);

    std::cout << "inliersPtr size: " << inliersPtr->indices.size() << std::endl;
    std::cout << "plane foeffs: " << coefficients.values[0] << " " << coefficients.values[1] << " " << coefficients.values[2] << " " << coefficients.values[3] << std::endl;

    return coefficients;
}

pcl::ModelCoefficients CloudPretreatmentEx::ParallelLine(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr,
    Eigen::Vector3f& axis ,
    const int& number)
{
    pcl::PointIndices::Ptr inliersPtr(new pcl::PointIndices);
    return ParallelLine(srcPointCloudPtr, inliersPtr, axis, number);
}

pcl::ModelCoefficients CloudPretreatmentEx::ParallelLine(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointIndices::Ptr& inliersPtr, 
    const Eigen::Vector3f& axis,
    const int& number)
{
    try
    {
        float epsAngle = _paramsConfig->parallelLine_epsAngle;
        float distanceThreshold = _paramsConfig->parallelLine_distanceThreshold;

        pcl::ModelCoefficients coefficients;
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PARALLEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setEpsAngle(epsAngle); //todo
        seg.setAxis(axis);
        seg.setDistanceThreshold(distanceThreshold); //todo
        seg.setInputCloud(srcPointCloudPtr);
        seg.segment(*inliersPtr, coefficients);

        if(inliersPtr->indices.size() < number)
        {
            pcl::ModelCoefficients lineCoeffsTmp;       
            return lineCoeffsTmp;
        }

        return coefficients;
    }
    catch(const std::exception& e)
    {
        std::string error = "Error in ParallelLine(): no model found.";
        throw std::runtime_error(error);
    } 
}

int CloudPretreatmentEx::ParallelLine(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointIndices::Ptr& inliersPtr, 
    const Eigen::Vector3f& axis,
    const int& number,
    pcl::ModelCoefficients& lineCoeffs)
{
    try
    {
        float epsAngle = _paramsConfig->parallelLine_epsAngle;
        float distanceThreshold = _paramsConfig->parallelLine_distanceThreshold;

        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PARALLEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setEpsAngle(epsAngle); //todo
        seg.setAxis(axis);
        seg.setDistanceThreshold(distanceThreshold); //todo
        seg.setInputCloud(srcPointCloudPtr);
        seg.segment(*inliersPtr, lineCoeffs);
        if(inliersPtr->indices.size() < number)
            return -1;
        return 0;
    }
    catch(const std::exception& e)
    {
        std::string error = "Error in ParallelLine(): no model found.";
        throw std::runtime_error(error);
    } 
}

pcl::ModelCoefficients CloudPretreatmentEx::Line(pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr)
{
    pcl::PointIndices::Ptr indicesPtr(new pcl::PointIndices);
    return Line(srcPointCloudPtr, indicesPtr);
}

pcl::ModelCoefficients CloudPretreatmentEx::Line(pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, pcl::PointIndices::Ptr& inliersPtr)
{
    float distanceThreshold = _paramsConfig->line_distanceThreshold;

    pcl::ModelCoefficients coefficients; 
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(srcPointCloudPtr);
    seg.segment(*inliersPtr, coefficients);

    return coefficients;
}

int CloudPretreatmentEx::PlaneSegmentation(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointCloud<pcl::Normal>::Ptr& PointCloudNormalPtr, 
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& pointCloudPtrVec,
    const int& engineering)
{
    int minClusterSize = _paramsConfig->planeSegmentation_minClusterSize;
    int MaxClusterSize = _paramsConfig->planeSegmentation_maxClusterSize;
    int numberOfNeighbours = _paramsConfig->planeSegmentation_numberOfNeighbours;
    float smoothnessThreshold = _paramsConfig->planeSegmentation_smoothnessThreshold;
    float curvatureThreshold = _paramsConfig->planeSegmentation_curvatureThreshold;

    switch (engineering)
    {
    case MASONRY:
        minClusterSize = _paramsConfig->masonryWall.planeSegmentation_minClusterSize;
        MaxClusterSize = _paramsConfig->masonryWall.planeSegmentation_maxClusterSize;
        numberOfNeighbours = _paramsConfig->masonryWall.planeSegmentation_numberOfNeighbours;
        smoothnessThreshold = _paramsConfig->masonryWall.planeSegmentation_smoothnessThreshold;
        curvatureThreshold = _paramsConfig->masonryWall.planeSegmentation_curvatureThreshold;
        break;    
    default:
        break;
    }

    pcl::search::Search<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> regions;

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> rg;
    rg.setMinClusterSize(minClusterSize); 
    rg.setMaxClusterSize(MaxClusterSize); 
    rg.setSearchMethod(tree);
    rg.setNumberOfNeighbours(numberOfNeighbours); 
    rg.setInputCloud(srcPointCloudPtr);
    rg.setInputNormals(PointCloudNormalPtr);
    rg.setSmoothnessThreshold(smoothnessThreshold); 
    rg.setCurvatureThreshold(curvatureThreshold); 
    rg.extract(regions);

    pointCloudPtrVec.resize(regions.size());

    for(size_t i = 0; i < regions.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr regionPointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointIndices::Ptr indicePtr(new pcl::PointIndices(regions[i]));
        Extract(srcPointCloudPtr, indicePtr, regionPointCloudPtr, false);
        pointCloudPtrVec[i] = std::move(regionPointCloudPtr);
    }

    return 0;
}

int CloudPretreatmentEx::PlaneSegmentation(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, 
    pcl::PointCloud<pcl::Normal>::Ptr& PointCloudNormalPtr, 
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& pointCloudPtrVec,
    std::vector<pcl::PointIndices::Ptr>& pointIndicesPtrVec,
    const int& engineering)
{
    int minClusterSize = _paramsConfig->planeSegmentation_minClusterSize;
    int MaxClusterSize = _paramsConfig->planeSegmentation_maxClusterSize;
    int numberOfNeighbours = _paramsConfig->planeSegmentation_numberOfNeighbours;
    float smoothnessThreshold = _paramsConfig->planeSegmentation_smoothnessThreshold;
    float curvatureThreshold = _paramsConfig->planeSegmentation_curvatureThreshold;

    switch (engineering)
    {
    case MASONRY:
        minClusterSize = _paramsConfig->masonryWall.planeSegmentation_minClusterSize;
        MaxClusterSize = _paramsConfig->masonryWall.planeSegmentation_maxClusterSize;
        numberOfNeighbours = _paramsConfig->masonryWall.planeSegmentation_numberOfNeighbours;
        smoothnessThreshold = _paramsConfig->masonryWall.planeSegmentation_smoothnessThreshold;
        curvatureThreshold = _paramsConfig->masonryWall.planeSegmentation_curvatureThreshold;
        break;    
    default:
        break;
    }

    pcl::search::Search<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> regions;

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> rg;
    rg.setMinClusterSize(minClusterSize); 
    rg.setMaxClusterSize(MaxClusterSize); 
    rg.setSearchMethod(tree);
    rg.setNumberOfNeighbours(numberOfNeighbours); 
    rg.setInputCloud(srcPointCloudPtr);
    rg.setInputNormals(PointCloudNormalPtr);
    rg.setSmoothnessThreshold(smoothnessThreshold); 
    rg.setCurvatureThreshold(curvatureThreshold); 
    rg.extract(regions);

    pointCloudPtrVec.resize(regions.size());
    pointIndicesPtrVec.resize(regions.size());

    for(size_t i = 0; i < regions.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr regionPointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointIndices::Ptr indicePtr(new pcl::PointIndices(regions[i]));
        Extract(srcPointCloudPtr, indicePtr, regionPointCloudPtr, false);
        pointCloudPtrVec[i] = std::move(regionPointCloudPtr);
        pointIndicesPtrVec[i] = std::move(indicePtr);
    }

    return 0; 
}

int CloudPretreatmentEx::ConcaveHull(pcl::PointCloud<pcl::PointXYZI>::Ptr& srcPointCloudPtr, pcl::PointCloud<pcl::PointXYZI>::Ptr& dstPointCloudPtr, const int& demension)
{
    float alpha = _paramsConfig->concaveHull_alpha;

    pcl::ConcaveHull<pcl::PointXYZI> chull;
    chull.setInputCloud (srcPointCloudPtr);
    chull.setAlpha (alpha);
    chull.setDimension (demension);
    chull.reconstruct (*dstPointCloudPtr);

    return 0;
}

int CloudPretreatmentEx::Viewer_PlaneClipper3D(pcl::PointCloud<pcl::PointXYZI>::Ptr& PointCloudPtr1, pcl::PointCloud<pcl::PointXYZI>::Ptr& PointCloudPtr2)
{
    pcl::visualization::PCLVisualizer::Ptr viewPtr(new pcl::visualization::PCLVisualizer("show_point_cloud"));

    int v1(0);
    viewPtr -> createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewPtr -> setBackgroundColor(0, 0, 0, v1);
    viewPtr -> addText("origin point cloud", 10 , 10, "v1_test", v1);

    int v2(0);
    viewPtr -> createViewPort(0.5 ,0.0, 0.5, 1.0, v2);
    viewPtr -> setBackgroundColor(0.1, 0.1, 0.1, v2);
    viewPtr -> addText("filter point cloud", 10, 10, "v2_test", v2);

    viewPtr -> addPointCloud<pcl::PointXYZI>(PointCloudPtr1, "origin point cloud", v1);
    viewPtr -> addPointCloud<pcl::PointXYZI>(PointCloudPtr2,"filter point cloud", v2);
    viewPtr -> setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0,"origin point cloud", v1);
    viewPtr -> setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0,"filter point cloud", v2);

    viewPtr -> addCoordinateSystem(0.1);

    while(!viewPtr->wasStopped())
    {
        viewPtr->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}






CloudPretreatmentEx::ParamsConfig::ParamsConfig() {}

CloudPretreatmentEx::ParamsConfig::ParamsConfig(const std::string& fileName)
{
    Load(fileName);    
    // PrintParam();
    Parse();
}

CloudPretreatmentEx::ParamsConfig::~ParamsConfig() {}

void CloudPretreatmentEx::ParamsConfig::Load(const std::string& fileName)
{
    std::ifstream file(fileName,std::ifstream::in);
    if(!file.is_open())
    {
        std::string error = "failed to open file: " + fileName;
        throw std::runtime_error(error);
    }
    node = YAML::Load(file);
}

void CloudPretreatmentEx::ParamsConfig::Parse()
{
    std::string item;
    try
    {
        int num = 0;
        // std::cout << "node: \n" << node << std::endl;
        YAML::Node normal_node = node["cloud_pretreatment_node"]["normal"];
        // std::cout << "num: " << ++num << std::endl;
        // std::cout << "normal_node:\n" << normal_node << std::endl;
        YAML::Node masonry_wall_node = node["cloud_pretreatment_node"]["masonry_wall"];
            // std::cout << "num: " << ++num << std::endl;
            // std::cout << "masonry_wall_node: \n" << masonry_wall_node << std::endl;

        /** normal **/
        {
        YAML::Node tmpNode = normal_node["statisticalFilter"];   
            // std::cout << "num: " << ++num << std::endl;
            // std::cout << "tmpNode: \n" << tmpNode << std::endl; 
        item = "statisticalFilter_meanK";
        statisticalFilter_meanK = tmpNode["statisticalFilter_meanK"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        item = "statisticalFilter_stddevMulThresh";
        statisticalFilter_stddevMulThresh = tmpNode["statisticalFilter_stddevMulThresh"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        }

        {
        YAML::Node tmpNode = normal_node["radiusFilter"];
            // std::cout << "num: " << ++num << std::endl;
        radiusFilter_radius = tmpNode["radiusFilter_radius"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        radiusFilter_minNeighborsInRadius = tmpNode["radiusFilter_minNeighborsInRadius"].as<int>();
            // std::cout << "num: " << ++num << std::endl;
        }
        {
        YAML::Node tmpNode = normal_node["voxelGridFilter"];
            // std::cout << "num: " << ++num << std::endl;
        voxelGridFilter_leafSize_x = tmpNode["voxelGridFilter_leafSize_x"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        voxelGridFilter_leafSize_y = tmpNode["voxelGridFilter_leafSize_y"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        voxelGridFilter_leafSize_z = tmpNode["voxelGridFilter_leafSize_z"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        }
        {
        YAML::Node tmpNode = normal_node["shadowPointsFilter"];
            // std::cout << "num: " << ++num << std::endl;
        shadowPointsFilter_threshold = tmpNode["shadowPointsFilter_threshold"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        }
        {
        YAML::Node tmpNode = normal_node["normalEstimation"];
            // std::cout << "num: " << ++num << std::endl;
        normalEstimation_numberOfThreads = tmpNode["normalEstimation_numberOfThreads"].as<int>();
            // std::cout << "num: " << ++num << std::endl;
        normalEstimation_kSearch = tmpNode["normalEstimation_kSearch"].as<int>();
            // std::cout << "num: " << ++num << std::endl;
        }
        {
        YAML::Node tmpNode = normal_node["plane"];
            // std::cout << "num: " << ++num << std::endl;
        plane_distanceThreshold = tmpNode["plane_distanceThreshold"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        }
        {
        YAML::Node tmpNode = normal_node["parallelLine"];
            // std::cout << "num: " << ++num << std::endl;
        parallelLine_epsAngle = tmpNode["parallelLine_epsAngle"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        parallelLine_distanceThreshold = tmpNode["parallelLine_distanceThreshold"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        }
        {
        YAML::Node tmpNode = normal_node["line"];
            // std::cout << "num: " << ++num << std::endl;
        line_distanceThreshold = tmpNode["line_distanceThreshold"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        }
        {
        YAML::Node tmpNode = normal_node["boundary"];
            // std::cout << "num: " << ++num << std::endl;
        boundary_radius = tmpNode["boundary_radius"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        boundary_angleThreshold = tmpNode["boundary_angleThreshold"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        }
        {
        YAML::Node tmpNode = normal_node["planeSegmentation"];
            // std::cout << "num: " << ++num << std::endl;
        planeSegmentation_minClusterSize = tmpNode["planeSegmentation_minClusterSize"].as<int>();
            // std::cout << "num: " << ++num << std::endl;
        planeSegmentation_maxClusterSize = tmpNode["planeSegmentation_maxClusterSize"].as<int>();
            // std::cout << "num: " << ++num << std::endl;
        planeSegmentation_numberOfNeighbours = tmpNode["planeSegmentation_numberOfNeighbours"].as<int>();
            // std::cout << "num: " << ++num << std::endl;
        planeSegmentation_smoothnessThreshold = tmpNode["planeSegmentation_smoothnessThreshold"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        planeSegmentation_curvatureThreshold = tmpNode["planeSegmentation_curvatureThreshold"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        }
        {
        YAML::Node tmpNode = normal_node["concaveHull"];
            // std::cout << "num: " << ++num << std::endl;
        concaveHull_alpha = tmpNode["concaveHull_alpha"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        }
        {
        /** masonry wall **/
        YAML::Node tmpNode = masonry_wall_node["normalEstimation"];
            // std::cout << "num: " << ++num << std::endl;
        masonryWall.normalEstimation_numberOfThreads = tmpNode["normalEstimation_numberOfThreads"].as<int>();
            // std::cout << "num: " << ++num << std::endl;
        masonryWall.normalEstimation_kSearch = tmpNode["normalEstimation_kSearch"].as<int>();
            // std::cout << "num: " << ++num << std::endl;
        }
        {
        YAML::Node tmpNode = masonry_wall_node["planeSegmentation"];
            // std::cout << "num: " << ++num << std::endl;
        masonryWall.planeSegmentation_minClusterSize = tmpNode["planeSegmentation_minClusterSize"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        masonryWall.planeSegmentation_maxClusterSize = tmpNode["planeSegmentation_maxClusterSize"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        masonryWall.planeSegmentation_numberOfNeighbours = tmpNode["planeSegmentation_numberOfNeighbours"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        masonryWall.planeSegmentation_smoothnessThreshold = tmpNode["planeSegmentation_smoothnessThreshold"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        masonryWall.planeSegmentation_curvatureThreshold = tmpNode["planeSegmentation_curvatureThreshold"].as<float>();
            // std::cout << "num: " << ++num << std::endl;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::cerr << "error in parse: " << item << std::endl;
    }
    catch (...)
    {
        std::cerr << "error in parse: " << item << std::endl;
    }
    

}

void CloudPretreatmentEx::ParamsConfig::PrintParam()
{
    std::cout 
        << std::endl
        << "/**** CloudPretreatmentEx params config ****/"
        << std::endl
        << std::endl
        << node 
        << std::endl 
        << std::endl;
}

}//namespace am