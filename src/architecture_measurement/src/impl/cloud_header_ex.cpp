#include "architecture_measurement/impl/cloud_header_ex.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <thread>
#include <fstream>

#include "pcl/features/moment_of_inertia_estimation.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/sample_consensus/sac_model_plane.h"

namespace am
{

CloudHeader::CloudHeader()
{

}

CloudHeader::CloudHeader(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _cloud, const int& _index) :  cloud(_cloud), index(_index)
{
    size = cloud -> points.size();
    std::thread th1(&CloudHeader::Plane, this);
    std::thread th2(&CloudHeader::CalculateOBB, this);
    th1.join();
    th2.join();   
}

CloudHeader::CloudHeader(const CloudHeader& cloud_header)
{
    cloud = nullptr;
    index = cloud_header.index;
    size = cloud_header.size;
    normalVector = cloud_header.normalVector;
    planeCoeff = cloud_header.planeCoeff;
    massCenter =cloud_header.massCenter;
}

CloudHeader& CloudHeader::operator=(const CloudHeader& cloud_header)
{
    if(this != &cloud_header)
    {
        this->cloud = cloud_header.cloud;
        this->index = cloud_header.index;
        this->size = cloud_header.size;
        this->normalVector = cloud_header.normalVector;
        this->planeCoeff = cloud_header.planeCoeff;
        this->massCenter = cloud_header.massCenter;
    }

    return *this;
}

// CloudHeader::CloudHeader(CloudHeader&& cloud_header)
// {
//     cloud = cloud_header.cloud;
//     index = cloud_header.index;
//     size = cloud_header.size;
//     normalVector = cloud_header.normalVector;
//     massCenter = cloud_header.massCenter;
// }

CloudHeader::~CloudHeader()
{
    cloud.reset();
}

// void CloudHeader::SetProperty(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _cloud, const int& _index)
// {
//     cloud = _cloud;
//     index = _index;

//     size = cloud -> points.size();
//     std::thread th1(&CloudHeader::Plane, this);
//     std::thread th2(&CloudHeader::CalculateOBB, this);
//     th1.join();
//     th2.join();

//     std::cout 
//         << "index: " << index 
//         << " size: " << size 
//         << " normalVector: " << normalVector.transpose() 
//         << " massCenter: " << massCenter.transpose() 
//         << std::endl;
// }

void CloudHeader::Plane()
{    
    std::unique_ptr<ParamsConfig> _paramsConfig = std::make_unique<ParamsConfig>("/home/ubuntu/am_v2_ws/install/architecture_measurement/share/architecture_measurement/config/cloud_header_params.yaml"); 
         
    float distanceThreshold = _paramsConfig->GetDistanceThreshold();
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coefficients;

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);//todo
    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);

    // std::cout << "coefficients: " << coefficients.values[0] << " " << coefficients.values[1] << " " << coefficients.values[2] << std::endl;
    normalVector << coefficients.values[0], coefficients.values[1], coefficients.values[2];
    planeCoeff << coefficients.values[0], coefficients.values[1], coefficients.values[2], coefficients.values[3];

    // std::cout << "cloud header plane coeff: " << planeCoeff.transpose() << std::endl;
    Eigen::Vector3f unitX{1.,0.,0.};
    if(normalVector.dot(unitX) < 0)
        normalVector = -normalVector;
}

void CloudHeader::CalculateOBB()
{
    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();
    feature_extractor.getMassCenter(massCenter);

}

/**** ParamsConfig ****/

CloudHeader::ParamsConfig::ParamsConfig()
{

}

CloudHeader::ParamsConfig::ParamsConfig(const std::string& fileName)
{
    Load(fileName);    
    // PrintParam();
    Parse();
}

CloudHeader::ParamsConfig::~ParamsConfig()
{

}

void CloudHeader::ParamsConfig::Load(const std::string& fileName)
{
    std::ifstream file(fileName,std::ifstream::in);
    if(!file.is_open())
        throw std::runtime_error("Error in Function ParamsConfig::Load(): file is invalid!");
    node = YAML::Load(file);
}

void CloudHeader::ParamsConfig::Parse()
{
    YAML::Node params_node = node["cloud_header_node"]["params"];

    YAML::Node plane_node = params_node["plane"];    
    distanceThreshold = plane_node["distanceThreshold"].as<float>();
}

void CloudHeader::ParamsConfig::PrintParam()
{
    std::cout 
        << "/**** CloudHeader params config ****/"
        << std::endl
        << std::endl
        << node 
        << std::endl 
        << std::endl;
}

/**** others ****/

bool SortSize(CloudHeader& ch1, CloudHeader& ch2)
{
    return ch1.GetSize() > ch2.GetSize();
}

bool SortNormalVectorL(CloudHeader& ch1, CloudHeader& ch2)
{
    Eigen::Vector3f unitZ{0.,0.,1.};
    float projectValue1 = std::fabs(ch1.GetNormalVector().dot(unitZ));
    float projectValue2 = std::fabs(ch2.GetNormalVector().dot(unitZ));
    return projectValue1 < projectValue2;
}

bool SortNormalVectorG(CloudHeader& ch1, CloudHeader& ch2)
{
    Eigen::Vector3f unitZ{0.,0.,1.};
    float projectValue1 = std::fabs(ch1.GetNormalVector().dot(unitZ));
    float projectValue2 = std::fabs(ch2.GetNormalVector().dot(unitZ));
    return projectValue1 > projectValue2;
}

bool SortMassCenterXL(CloudHeader& ch1, CloudHeader& ch2)
{
    const auto& massCenter1 = ch1.GetMassCenter();
    const auto& massCenter2 = ch2.GetMassCenter();
    return massCenter1(0) < massCenter2(0);
}

bool SortMassCenterXG(CloudHeader& ch1, CloudHeader& ch2)
{
    const auto& massCenter1 = ch1.GetMassCenter();
    const auto& massCenter2 = ch2.GetMassCenter();
    return massCenter1(0) > massCenter2(0);
}

bool SortMassCenterYL(CloudHeader& ch1, CloudHeader& ch2)
{
    const auto& massCenter1 = ch1.GetMassCenter();
    const auto& massCenter2 = ch2.GetMassCenter();
    return massCenter1(1) < massCenter2(1);
}

bool SortMassCenterYG(CloudHeader& ch1, CloudHeader& ch2)
{
    const auto& massCenter1 = ch1.GetMassCenter();
    const auto& massCenter2 = ch2.GetMassCenter();
    return massCenter1(1) > massCenter2(1);
}

bool SortMassCenterZL(CloudHeader& ch1, CloudHeader& ch2)
{
    const auto& massCenter1 = ch1.GetMassCenter();
    const auto& massCenter2 = ch2.GetMassCenter();
    return massCenter1(2) < massCenter2(2);
}

bool SortMassCenterZG(CloudHeader& ch1, CloudHeader& ch2)
{
    const auto& massCenter1 = ch1.GetMassCenter();
    const auto& massCenter2 = ch2.GetMassCenter();
    return massCenter1(2) > massCenter2(2);
}

} //namespace am



