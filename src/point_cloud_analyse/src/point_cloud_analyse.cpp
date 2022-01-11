// Copyright 2019 Zhushi Tech, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "point_cloud_analyse/point_cloud_analyse.hpp"

#include <algorithm>
#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/features/normal_3d.h"
#include "pcl/segmentation/region_growing.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/sample_consensus/sac_model_plane.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/moment_of_inertia_estimation.h"

namespace point_cloud_analyse
{

void Save(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const std::string & fileName)
{
  std::ofstream ofile(fileName);
  ofile << std::fixed;
  for (const auto & p : *cloud) {
    ofile <<
      p.x * 1000 << ' ' <<
      p.y * 1000 << ' ' <<
      p.z * 1000 << ' ' <<
      p.intensity * 1000 << '\n';
  }
}

class PointCloudAnalyse::_Impl
{
public:
  explicit _Impl(PointCloudAnalyse * ptr) : _node(ptr)
  {
        std::vector<double> r_temp[2];
        
        _absDiff[0].resize(2);
        _absDiff[1].resize(2);
        r_temp[0].resize(9);
        r_temp[1].resize(9);
        
        _node->declare_parameter("r_sidewall");
        _node->declare_parameter("abs_diff_sidewall");
        _node->declare_parameter("r_ground");
        _node->declare_parameter("abs_diff_ground");
        
        _node->get_parameter("abs_diff_sidewall",_absDiff[0]);
        _node->get_parameter("r_sidewall",r_temp[0]);
        _node->get_parameter("abs_diff_ground",_absDiff[1]);
        _node->get_parameter("r_ground",r_temp[1]);
        
        _rAtt[0] << r_temp[0].at(0),r_temp[0].at(1),r_temp[0].at(2),r_temp[0].at(3),
                    r_temp[0].at(4),r_temp[0].at(5),r_temp[0].at(6),r_temp[0].at(7),
                    r_temp[0].at(8);
        _rAtt[1] << r_temp[1].at(0),r_temp[1].at(1),r_temp[1].at(2),r_temp[1].at(3),
                    r_temp[1].at(4),r_temp[1].at(5),r_temp[1].at(6),r_temp[1].at(7),
                    r_temp[1].at(8);    
  }

  ~_Impl()
  {
  }

  pcl::ModelCoefficients Plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointIndices & inliers)
  {
    pcl::ModelCoefficients coefficients;
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);

    return coefficients;
  }

  pcl::ModelCoefficients Plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    pcl::PointIndices inliers;
    return Plane(cloud, inliers);
  }

  void StatisticalRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr res)
  {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(10);
    sor.setStddevMulThresh(1.0);
    sor.filter(*res);

    return;
  }

  void RadiusRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr res)
  {
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> rad;
    rad.setInputCloud(cloud);
    rad.setRadiusSearch(0.05);
    rad.setMinNeighborsInRadius(9);
    rad.filter(*res);

    return;
  }

  void ConditionRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr& res, const char* field, double l_limit, double h_limit)
  {
    pcl::ConditionAnd<pcl::PointXYZI>::Ptr cond(new pcl::ConditionAnd<pcl::PointXYZI>());
    cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>(field,pcl::ComparisonOps::GT,l_limit)));
    cond -> addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>(field,pcl::ComparisonOps::LT,h_limit)));

    pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
    condrem.setInputCloud(cloud);
    condrem.setCondition(cond);
    condrem.setKeepOrganized(true);
    condrem.filter(*res);

    return;

  }

  void Segment(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<pcl::PointIndices> & regions)
  {
    // Downsample
    /*pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01, 0.2, 0.01);
    vg.filter(*cloud_grid);*/

    // Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    pcl::search::Search<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(25);
    ne.compute(*normal);

    // pcl::io::savePCDFileASCII("/home/ubuntu/http_server/cloud.pcd", *cloud);

    // Region grow
    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> rg;
    rg.setMinClusterSize(100);
    rg.setMaxClusterSize(1000000);
    rg.setSearchMethod(tree);
    rg.setNumberOfNeighbours(25);
    rg.setInputCloud(cloud);
    rg.setInputNormals(normal);
    rg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
    rg.setCurvatureThreshold(1.0);
    rg.extract(regions);

    // Sort region
    for (size_t i = 0; i < regions.size(); ++i) {
      auto & ri = regions[i];
      for (size_t j = i + 1; j < regions.size(); ++j) {
        auto & rj = regions[j];
        if (rj.indices.size() > ri.indices.size()) {
          std::swap(ri, rj);
        }
      }
    }

    for (size_t i = 0; i < regions.size(); ++i) {
      for (auto j : regions[i].indices) {
        (*cloud)[j].intensity = i;
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr Extract(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointIndices::Ptr indicePtr)
  {
    pcl::ExtractIndices<pcl::PointXYZI> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(indicePtr);
    // RCLCPP_INFO( _node -> get_logger(),"[Extract]: cloud size: %d , indices size: %d", cloud->size(), indicePtr->indices.size());
    pcl::PointCloud<pcl::PointXYZI>::Ptr ret(new pcl::PointCloud<pcl::PointXYZI>);
    ei.filter(*ret);
    return ret;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr Extract(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointIndices & indice)
  {
    pcl::PointIndices::Ptr indicePtr(new pcl::PointIndices(indice));
    return Extract(cloud, indicePtr);
  }

  std::pair<double, double> Statistic(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    if (cloud->width == 0) {return {0, 0};}

    std::vector<double> v;
    v.reserve(cloud->width);

    for (const auto & p : *cloud) {v.push_back(p.intensity);}
    auto sum = std::accumulate(v.begin(), v.end(), 0.0);
    auto mean = sum / v.size();
    std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(), [mean](double x) {return x - mean;});
    auto dev = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.) / v.size();

    return {mean, std::sqrt(dev)};
  }

  std::vector<double> VirtualRuler(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    const int L = 4;
    std::vector<pcl::PointIndices> lines(L);

    const double P[L][4] = {
      {0, 0, 1, 0},
      {1, 0, 0, 0},      
      {std::sqrt(0.5), 0, std::sqrt(0.5), 0},
      {std::sqrt(0.5), 0, -std::sqrt(0.5), 0}
    };

    for (uint32_t i = 0; i < cloud->width; ++i) {
      const auto & p = cloud->at(i);
      for (int j = 0; j < L; ++j) {
        auto d = std::abs(p.x * P[j][0] + p.y * P[j][1] + p.z * P[j][2] + P[j][3]);
        if (d < 0.02) {lines[j].indices.push_back(i);}
      }
    }

    auto line0 = Extract(cloud, lines[0]);
    // Save(line0, "/home/ubuntu/http_server/test_pcd0.txt");
    std::pair<double, double> s0 = Statistic(line0);

    auto line1 = Extract(cloud, lines[1]);
    // Save(line1, "/home/ubuntu/http_server/test_pcd1.txt");
    std::pair<double, double> s1 = Statistic(line1);

    auto line2 = Extract(cloud, lines[2]);
    std::pair<double, double> s2 = Statistic(line2);

    auto line3 = Extract(cloud, lines[3]);
    std::pair<double, double> s3 = Statistic(line3);

    RCLCPP_INFO(_node -> get_logger(),"virtual ruler:  s0.first: %f    s0.second: %f    s1.first: %f    s1.second: %f   s2.first: %f    s2.second: %f   s3.first: %f    s3.second: %f",
                                                      s0.first * 1000, s0.second * 1000,
                                                      s1.first * 1000, s1.second * 1000,
                                                      s2.first * 1000, s2.second * 1000,
                                                      s3.first * 1000, s3.second * 1000);

    return {
/* *INDENT-OFF* */
      s0.first * 1000, s0.second * 1000,
      s1.first * 1000, s1.second * 1000,
      s2.first * 1000, s2.second * 1000,
      s3.first * 1000, s3.second * 1000
/* *INDENT-ON* */
    };
  }

  /**
   * @brief feature segmentation for accuracy test
  */
  pcl::ModelCoefficients::Ptr SegmentT(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointIndices::Ptr inliers,float DistanceThreshold,int flag)
  {      
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);   
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    switch(flag)
    {
      case TEST_STRAIGHTNESS:
        seg.setModelType(pcl::SACMODEL_LINE);
        break;
      case TEST_PLANEFLATNESS:
        seg.setModelType(pcl::SACMODEL_PLANE);
        break;
      default:
        throw std::runtime_error("[point_cloud_analyse]: function \"SegmentT\": unvalid flag");
    }
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(DistanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    
    return coefficients;
  }


  void FilterByIntensity(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr &res, double threshold)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZI>);    
    line_cloud -> width = cloud -> size();
    line_cloud -> height = 1;
    line_cloud -> resize(line_cloud -> width * line_cloud -> height);       

    for(size_t i = 0; i < cloud -> size(); i++)
    {
      auto &p = (*cloud)[i];
      auto &lp = (*line_cloud)[i];
      lp.x = i;
      lp.y = p.intensity;
      lp.z = 0.;
      lp.intensity = 0.;
    }
    
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coef =  SegmentT(line_cloud,inliers,threshold,TEST_STRAIGHTNESS);
    res = Extract(cloud,inliers);
    return;
  }

  std::vector<double> CalculatePlaneFlat(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    Save(cloud, "/home/ubuntu/http_server/test_pcd_ori.txt");
    // RCLCPP_INFO(_node -> get_logger(), "[calculatePlaneFlat]: cloud size: %d",cloud->size());
    // pcl::PointCloud<pcl::PointXYZI>::Ptr sorCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr radCloud(new pcl::PointCloud<pcl::PointXYZI>); 
    // StatisticalRemoval(cloud,sorCloud);
    RadiusRemoval(cloud,radCloud);
    // RCLCPP_INFO(_node -> get_logger(), "[calculatePlaneFlat]: sorCloud size: %d",sorCloud->size());
    // RCLCPP_INFO(_node -> get_logger(), "[calculatePlaneFlat]: sorCloud size: %d",radCloud->size());
    // Save(sorCloud, "/home/ubuntu/http_server/test_pcd2.txt");
    Save(radCloud, "/home/ubuntu/http_server/test_pcd_rad.txt");
    

    std::vector<pcl::PointIndices> regions;
    Segment(radCloud, regions);
    if (regions.empty()) {
      Save(radCloud, "/home/ubuntu/http_server/test_pcd5.txt");
      throw std::runtime_error("CalculatePlaneFlat: no planes");
    }

    // Extract indices
    auto cloud0 = Extract(radCloud, regions[0]);

    // Fit plane
    auto c0 = Plane(cloud0);

    for (auto & p : *cloud0)
    {
      p.intensity = pcl::pointToPlaneDistanceSigned<pcl::PointXYZI>(
        p,
        c0.values[0],
        c0.values[1],
        c0.values[2],
        c0.values[3]);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr conCloud(new pcl::PointCloud<pcl::PointXYZI>);
    FilterByIntensity(cloud0,conCloud,0.002);
    // RCLCPP_INFO(_node -> get_logger(), "conCloud size: %d" ,conCloud->size());
    // ConditionRemoval(cloud0,conCloud,-2.0,2.0);
    Save(cloud0, "/home/ubuntu/http_server/test_pcd5.txt");
    Save(conCloud, "/home/ubuntu/http_server/test_pcd_con.txt");
    
    std::vector<double> v;
    v.reserve(conCloud->width);

    for (const auto & p : *conCloud) {v.push_back(p.intensity);}
    auto sum = std::accumulate(v.begin(), v.end(), 0.0);
    auto mean = sum / v.size();
    // RCLCPP_INFO(_node -> get_logger(), "[CalculatePlaneFlat]: mean: %f", mean);
    std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(), [mean](double x) {return x - mean;});
    auto dev = std::sqrt(std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.) / (v.size()-1));
    RCLCPP_INFO(_node -> get_logger(), "[CalculatePlaneFlat]: dev: %f", dev);//todo


    /*auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud0, *msg);
    msg->header.stamp = _node->now();
    msg->header.frame_id = _node->_frameID;
    _node->_pub->publish(std::move(msg));*/

    // return VirtualRuler(conCloud);

    return {mean, dev, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

    std::vector<double> CalculatePlaneHV(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, bool vertical)
    {
        //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_grid(new pcl::PointCloud<pcl::PointXYZI>);
        std::vector<pcl::PointIndices> regions;
        Segment(cloud, regions);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CalculatePlaneHV: Segment region size: %d", regions.size());

        if(regions.empty())
        {
            Save(cloud, "/home/ubuntu/http_server/test_pcd5.txt");
            throw std::runtime_error("CalculatePlaneFlat: no planes");
        }

        //Extract indices
        auto cloud0 = Extract(cloud, regions[0]);

        //Fit plane
        auto c0 = Plane(cloud0);//TODO
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "plane: %f,%f,%f,%f",  c0.values[0], c0.values[1], c0.values[2], c0.values[3]);

        for(auto& p : *cloud0)
        {
            p.intensity = pcl::pointToPlaneDistanceSigned<pcl::PointXYZI>(p, c0.values[0], c0.values[1], c0.values[2], c0.values[3]);
        }

        Save(cloud0, "/home/ubuntu/http_server/test_pcd5.txt");

        Eigen::Affine3f t,r;
        
        if(vertical)
        {
            pcl::getTransformation(0., 0., 0., -_node->_roll +_absDiff[0].at(0), -_node->_pitch +_absDiff[0].at(1), _node->_yaw, t);
            r = _rAtt[0];
        }
        else
        {
            pcl::getTransformation(0., 0., 0.,  -_node->_roll+_absDiff[1].at(0), -_node->_pitch+_absDiff[1].at(1), _node->_yaw, t);
            r = _rAtt[1];
        }
        
        Eigen::Vector3f v(c0.values[0], c0.values[1], c0.values[2]); 
        
        std::cout<< "r_test" << r.linear() << std::endl;
        std::cout<< "t_test" << t.linear() << std::endl;
        
        auto c1 = t * v;
        

        //auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        //pcl::toROSMsg(*cloud0, *msg);
        //msg->header.stamp = _node->now();
        //msg->header.frame_id = _node->_frameID;
        //_node->_pub->publish(std::move(msg));

        auto ret = asin(c1(2)) / M_PI * 180.;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "angle: %f",  ret);

        if(vertical)  
            return {abs(ret)};
        else
            return {90. - abs(ret)};
    }

  std::vector<double> CalculateIncludedAngle(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    std::vector<pcl::PointIndices> regions;
    Segment(cloud, regions);

    if (regions.size() < 2) {
      Save(cloud, "/home/ubuntu/http_server/test_pcd5.txt");
      throw std::runtime_error("CalculateIncludedAngle: not enough planes");
    }

    // Extract indices
    auto cloud0 = Extract(cloud, regions[0]);
    auto cloud1 = Extract(cloud, regions[1]);

    // Fit plane
    auto c0 = Plane(cloud0);
    auto c1 = Plane(cloud1);

    *cloud0 += *cloud1;
    Save(cloud0, "/home/ubuntu/http_server/test_pcd5.txt");

    /*auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud0, *msg);
    msg->header.stamp = _node->now();
    msg->header.frame_id = _node->_frameID;
    _node->_pub->publish(std::move(msg));*/

    // Calculate included angle
    auto rd =
      c0.values[0] * c1.values[0] +
      c0.values[1] * c1.values[1] +
      c0.values[2] * c1.values[2];
    return {acos(rd) / M_PI * 180.};
  }

  std::vector<double> CalculateOBB(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,int flag)
  {
    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    pcl::PointXYZI min_point_OBB,max_point_OBB,position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    float major_value, middle_value, minor_value; 
    Eigen::Vector3f major_vector, middle_vector, minor_vector;

    // Eigen::Vector3f mass_center; 

    feature_extractor.getOBB(min_point_OBB,max_point_OBB, position_OBB, rotational_matrix_OBB); 
    feature_extractor.getEigenValues(major_value,middle_value, minor_value); 
    feature_extractor.getEigenVectors(major_vector,middle_vector, minor_vector); 
    // feature_extractor.getMassCenter(mass_center);

    std::vector<double> deviation;
    Eigen::Vector4f plane{0.,0.,0.,0.};
    float d = 0.;
    switch(flag)
    {
      case TEST_STRAIGHTNESS:
        d = - middle_vector[0]*position_OBB.x - middle_vector[1]*position_OBB.y - middle_vector[2]*position_OBB.z;
        plane = {middle_vector[0],middle_vector[1],middle_vector[2],d};
        break;
      case TEST_PLANEFLATNESS:
        d = - minor_vector[0]*position_OBB.x - minor_vector[1]*position_OBB.y - minor_vector[2]*position_OBB.z;
        plane = {minor_vector[0],minor_vector[1],minor_vector[2],d};
        break;
      default:
        throw std::runtime_error("error: [point_cloud_analyse]: function \"CalculateOBB\", invalid flag");
    }

    for(const auto& p : *cloud)
    {
      Eigen::Vector4f point = {p.x,p.y,p.z,1.};
      auto mol = point.dot(plane);
      auto den = middle_vector.norm();

      deviation.emplace_back(mol/den);
    }

    auto minmax_pair = std::minmax_element(deviation.begin(),deviation.end());
    auto result = *minmax_pair.second - *minmax_pair.first;

    return {result};
  }

  std::vector<double> CalculateStraightnessT(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);    
    // pcl::PointCloud<pcl::PointXYZI>::Ptr res = Extract(cloud,inliers);
    pcl::PointCloud<pcl::PointXYZI>::Ptr resz(new pcl::PointCloud<pcl::PointXYZI>);
    ConditionRemoval(cloud,resz,"z",_node -> _straightness_z_l,_node -> _straightness_z_h);
    Save(resz, "/home/ubuntu/http_server/straightness_cloud.txt");
    pcl::ModelCoefficients::Ptr coef =  SegmentT(resz,inliers,0.1,TEST_STRAIGHTNESS);
    // Save(res, "/home/ubuntu/http_server/test_pcd5.txt");
    auto result = CalculateOBB(resz,TEST_STRAIGHTNESS);

    Eigen::Vector3f originPoint(coef -> values[0], coef -> values[1], coef -> values[2]); 
    Eigen::Vector3f lineDirection(coef -> values[3], coef -> values[4], coef -> values[5]);
       
    float dist = 0.;
    for(auto& p : *resz)
    {
      Eigen::Vector3f pointDirection{p.x - originPoint[0], p.y - originPoint[1], p.z - originPoint[2]};
      Eigen::Vector3f crossVec(lineDirection.cross(pointDirection));      
      p.intensity =   crossVec.norm() / lineDirection.norm(); //float sinTheta = crossVec.norm() / lineDirection.norm() / pointDirection.norm();
      if(p.intensity > dist )
      {
        dist = p.intensity;
      }
    }

    result.emplace_back(dist);
    return result;
  }

  std::vector<double> CalculatePlaneFlatnessT(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  
    // pcl::PointCloud<pcl::PointXYZI>::Ptr res = Extract(cloud,inliers);
    pcl::PointCloud<pcl::PointXYZI>::Ptr resz(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr reszx(new pcl::PointCloud<pcl::PointXYZI>);
    ConditionRemoval(cloud,resz,"z",_node -> _planeflatness_z_l,_node -> _planeflatness_z_h);
    ConditionRemoval(resz,reszx,"x",_node -> _planeflatness_x_l,_node -> _planeflatness_x_h);
    Save(reszx, "/home/ubuntu/http_server/planeflatness_cloud.txt");

    pcl::ModelCoefficients::Ptr coef =  SegmentT(reszx,inliers,0.1,TEST_PLANEFLATNESS);
    // Save(res, "/home/ubuntu/http_server/test_pcd5.txt");

    auto result = CalculateOBB(reszx,TEST_PLANEFLATNESS);

    float max = 0.;
    float min = 0.;
    for (auto & p : *reszx)
    {
      p.intensity = pcl::pointToPlaneDistanceSigned<pcl::PointXYZI>(
        p,
        coef -> values[0],
        coef -> values[1],
        coef -> values[2],
        coef -> values[3]);

      if(max < p.intensity)
        max = p.intensity;
      if(min > p.intensity)
        min = p.intensity;
    }

    result.emplace_back(max - min);

    return result;
  }

private:
  enum FLAG {TEST_STRAIGHTNESS,TEST_PLANEFLATNESS};

  PointCloudAnalyse * _node;

  std::mutex _mutex;              ///< Mutex to protect shared storage
  std::condition_variable _con;   ///< Conditional variable rely on mutex
  std::deque<sensor_msgs::msg::Image::UniquePtr> _deq;
  std::thread _thread;
  
  std::vector<double> _absDiff[2];   ///Absolute deviation of inclinometer
  Eigen::Matrix3f   _rAtt[2];    ///Rotation matrix of inclinometer
};


/**
 *
*/
PointCloudAnalyse::PointCloudAnalyse(const rclcpp::NodeOptions & options) : Node("point_cloud_analyse_node", options)
{
  _init = std::thread(&PointCloudAnalyse::_Init, this);
}

PointCloudAnalyse::~PointCloudAnalyse()
{
  _init.join();

  _sub.reset();
  _impl.reset();

  RCLCPP_INFO(this->get_logger(), "point_cloud_analyse destroyed successfully");//todo
}

void PointCloudAnalyse::_Init()
{
  try {
    _InitializeParameters();

    _UpdateParameters();

    _pubResult = this->create_publisher<shared_interfaces::msg::Float64Array>(_pubResultName, 1);

    _impl = std::make_unique<_Impl>(this);

    _sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      _subName,
      1,
      std::bind(&PointCloudAnalyse::_Sub, this, std::placeholders::_1)
    );

    _subRpy = this->create_subscription<shared_interfaces::msg::RollPitchYaw>(
      _subRpyName,
      10,
      [this](shared_interfaces::msg::RollPitchYaw::UniquePtr ptr)
      {
        _roll = ptr->roll;
        _pitch = ptr->pitch;
        _yaw = ptr->yaw;
      }
    );

    RCLCPP_INFO(this->get_logger(), "Initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in initializer: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in initializer: unknown");
  }
}


void PointCloudAnalyse::_Sub(sensor_msgs::msg::PointCloud2::UniquePtr ptr)
{
  try
  {  
    std::chrono::steady_clock::time_point start_point = std::chrono::steady_clock::now();
    if(!ptr -> width)
    {
      auto msg = std::make_unique<shared_interfaces::msg::Float64Array>();
      _pubResult->publish(std::move(msg));
      return;
    }
    _UpdateParameters();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::moveFromROSMsg(*ptr, *cloud);

    auto msg = std::make_unique<shared_interfaces::msg::Float64Array>();
    std::cout<<"_mode: "<<_mode<<std::endl;
    switch (_mode)
    {
      case 0:
        msg->data = _impl->CalculatePlaneFlat(cloud);
        break;
      case 1:
        msg->data = _impl->CalculateIncludedAngle(cloud);
        break;
      case 2:
        msg->data = _impl->CalculatePlaneHV(cloud, false);
        break;
      case 3:
        msg->data = _impl->CalculatePlaneHV(cloud, true);
        break;
      case 4:
        
        msg -> data = _impl->CalculateStraightnessT(cloud);
        break;
      case 5:
        msg -> data = _impl->CalculatePlaneFlatnessT(cloud);
        break;
      default:
        throw std::runtime_error("Unsupported operation");
    }

    _pubResult->publish(std::move(msg));

    std::chrono::steady_clock::time_point end_point = std::chrono::steady_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_point - start_point);
    std::cout << "[point_cloud_analyse] spend time: " << duration.count() << "ms" << std::endl;

  } 
  catch (const std::exception & e)
  {
    auto msg = std::make_unique<shared_interfaces::msg::Float64Array>();
    _pubResult->publish(std::move(msg));
    RCLCPP_WARN(this->get_logger(), "Exception in subscription: %s", e.what());
  } 
  catch (...)
  {
    auto msg = std::make_unique<shared_interfaces::msg::Float64Array>();
    _pubResult->publish(std::move(msg));
    RCLCPP_WARN(this->get_logger(), "Exception in subscription: unknown");
  }
}

void PointCloudAnalyse::_InitializeParameters()
{
  declare_parameter("frame_id");
  declare_parameter("mode");

  declare_parameter("straightness_z_l");
  declare_parameter("straightness_z_h");
  declare_parameter("planeflatness_z_l");
  declare_parameter("planeflatness_z_h");
  declare_parameter("planeflatness_x_l");
  declare_parameter("planeflatness_x_h");

}

void PointCloudAnalyse::_UpdateParameters()
{
  get_parameter("frame_id", _frameID);
  get_parameter("mode", _mode);

  get_parameter("straightness_z_l", _straightness_z_l);
  get_parameter("straightness_z_h", _straightness_z_h);
  get_parameter("planeflatness_z_l", _planeflatness_z_l);
  get_parameter("planeflatness_z_h", _planeflatness_z_h);
  get_parameter("planeflatness_x_l", _planeflatness_x_l);
  get_parameter("planeflatness_x_h", _planeflatness_x_h);

}

}  // namespace point_cloud_analyse

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(point_cloud_analyse::PointCloudAnalyse)
