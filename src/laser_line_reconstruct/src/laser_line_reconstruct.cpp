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

#include "laser_line_reconstruct/laser_line_reconstruct.hpp"

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pcl_conversions/pcl_conversions.h"
#include "opencv2/opencv.hpp"

namespace laser_line_reconstruct
{

using shared_interfaces::msg::LineCenter;
using sensor_msgs::msg::PointCloud2;

class LaserLineReconstruct::_Impl
{
public:
  explicit _Impl(LaserLineReconstruct * ptr)
  : _node(ptr)
  {
    _InitializeParameters();

    _UpdateParameters();

    std::vector<double> c[2], d[2], r[2], p[2], q;
    _node->get_parameter("camera_matrix_l", c[0]);
    _node->get_parameter("camera_matrix_r", c[1]);
    _node->get_parameter("dist_coeffs_l", d[0]);
    _node->get_parameter("dist_coeffs_r", d[1]);
    _node->get_parameter("rect_l", r[0]);
    _node->get_parameter("rect_r", r[1]);
    _node->get_parameter("proj_l", p[0]);
    _node->get_parameter("proj_r", p[1]);
    _node->get_parameter("q", q);

    _c[0] = cv::Mat(3, 3, CV_64F, c[0].data()).clone();
    _c[1] = cv::Mat(3, 3, CV_64F, c[1].data()).clone();

    _d[0] = cv::Mat(1, 5, CV_64F, d[0].data()).clone();
    _d[1] = cv::Mat(1, 5, CV_64F, d[1].data()).clone();

    _r[0] = cv::Mat(3, 3, CV_64F, r[0].data()).clone();
    _r[1] = cv::Mat(3, 3, CV_64F, r[1].data()).clone();

    _p[0] = cv::Mat(3, 4, CV_64F, p[0].data()).clone();
    _p[1] = cv::Mat(3, 4, CV_64F, p[1].data()).clone();

    _q = cv::Mat(4, 4, CV_64F, q.data()).clone();

    _thread = std::thread(&_Impl::_Worker, this);
  }

  ~_Impl()
  {
    _con.notify_all();
    _thread.join();
  }

  void PushBackL(LineCenter::UniquePtr ptr)
  {
    std::unique_lock<std::mutex> lk(_mutex);
    // if(ptr -> header.frame_id == "-1")
    // {
    //   // RCLCPP_INFO(rclcpp::get_logger("laser_line_reconstruct")," subscribe a lineL, header.frame_id = -1");//todo
    // }
    _deqL.emplace_back(std::move(ptr));
    lk.unlock();
    _con.notify_all();
  }

  void PushBackR(LineCenter::UniquePtr ptr)
  {
    std::unique_lock<std::mutex> lk(_mutex);
    // if(ptr -> header.frame_id == "-1")
    // {
    //   // RCLCPP_INFO(rclcpp::get_logger("laser_line_reconstruct")," subscribe a lineR, header.frame_id = -1");//todo
    // }
    _deqR.emplace_back(std::move(ptr));
    lk.unlock();
    _con.notify_all();
  }

private:
  PointCloud2::UniquePtr _Execute(
    const LineCenter::UniquePtr & ptrL,
    const LineCenter::UniquePtr & ptrR)
  {
    const auto & centerL = ptrL->center;
    const auto & centerR = ptrR->center;
    // std::cout << "[" << _node->get_name() << "]" << ": centerL size: " << centerL.size() << "; centerR size: " << centerR.size() <<std::endl;
    // cv::waitKey(5);

    _pL.clear();
    _pR.clear();
    _pnts.clear();
    _uv.clear();

    for (size_t i = 0; i < centerL.size(); ++i) {
      if (centerL[i] >= 0) {
        _pL.emplace_back(centerL[i], i);
      }
    }

    for (size_t i = 0; i < centerR.size(); ++i) {
      if (centerR[i] >= 0) {
        _pR.emplace_back(centerR[i], i);
      }
    }

    if (_pL.empty() || _pR.empty()) 
    {
      // std::cout<< "frame_id:" <<ptrL->header.frame_id<< " _pL.empty() || _pR.empty()" << std::endl;
      return nullptr;
    }

    cv::undistortPoints(_pL, _unpL, _c[0], _d[0], _r[0], _p[0]);
    cv::undistortPoints(_pR, _unpR, _c[1], _d[1], _r[1], _p[1]);

    // <y, x>
    std::multimap<float, float> mmap;
    for (size_t i = 0; i < _unpR.size(); ++i)
    {
      mmap.insert({_unpR[i].y, _unpR[i].x});
    }

    for (size_t i = 0, j = 0; i < centerL.size() - 1; ++i)
    {
      if (centerL[i] < 0) {continue;}

      size_t k = j++;
      if (centerL[i + 1] < 0) {continue;}

      const auto & a = _unpL[k];
      const auto & b = _unpL[k + 1];

      // Discard a.y >= b.y cases
      if (a.y >= b.y) {continue;}

      auto lower = mmap.lower_bound(a.y);
      auto upper = mmap.upper_bound(b.y);

      if (lower == upper)
      {
        continue;
      } 
      else
      {
        auto x = _InterpolateX(a, b, lower->first);
        _pnts.emplace_back(x, lower->first, x - lower->second);
        _uv.emplace_back(static_cast<int>(centerL[i]), static_cast<int>(i));
      }
    }

    if (_pnts.empty())
    {
      return nullptr;
    } 
    else 
    {
      std::vector<cv::Point3f> temp;
      for (size_t i = 0; i < _pnts.size(); i += 10)
      {
        temp.push_back(_pnts[i]);
      }

      std::swap(temp, _pnts);
      cv::perspectiveTransform(_pnts, _pnts, _q);

      pcl::PointCloud<pcl::PointXYZI> cloud;
      cloud.height = 1;
      cloud.width = _pnts.size();
      cloud.points.resize(_pnts.size());

      for (size_t i = 0; i < _pnts.size(); ++i) {
        cloud[i].x = _pnts[i].x;
        cloud[i].y = _pnts[i].z;
        cloud[i].z = -_pnts[i].y;
        cloud[i].intensity = _uv[i].y;
      }

      auto ptr = std::make_unique<PointCloud2>();
      pcl::toROSMsg(cloud, *ptr);
      ptr->header.stamp = ptrL->header.stamp;
      ptr->header.frame_id = _frameID;
      ptr->header.frame_id = ptrL->header.frame_id;

      return ptr;
    }
  }

  void _Worker()
  {
    std::chrono::steady_clock::time_point start_point;
    bool test_time = true;
    std::string frameId = "-1";

    while (rclcpp::ok()) 
    {
      std::unique_lock<std::mutex> lk(_mutex);

      LineCenter::UniquePtr pL = nullptr;
      LineCenter::UniquePtr pR = nullptr;
      if (_Pair(pL, pR))
      {
        if(test_time)
        {
          start_point = std::chrono::steady_clock::now();
          test_time = false;
        }
        lk.unlock();
        // RCLCPP_INFO(rclcpp::get_logger("laser_line_reconstruct"),"PL frame_id: %s",pL->header.frame_id.c_str());//todo
        if ( !(pL->header.frame_id == "-1" || pR->header.frame_id == "-1") )
        {
          auto ptr = _Execute(pL, pR);
         
          if (ptr) 
          {
            // std::cout<<"[laser_line_reconstruct] frame_id: " <<  ptr->header.frame_id <<std::endl;
            frameId = ptr->header.frame_id;
            // RCLCPP_INFO(rclcpp::get_logger("laser_line_reconstruct")," publish line pointcloud,frame_id: %s",ptr -> header.frame_id.c_str());//todo
            _node->Publish(ptr);


          }
          // RCLCPP_INFO(rclcpp::get_logger("laser_line_reconstruct")," publish line pointcloud, header.frame_id = -1");//todo
        }
        else 
        {    
            std::cout << "[" << _node->get_name() <<"]" << ": frameId: " << frameId << std::endl;
            auto ptr = std::make_unique<PointCloud2>();
            if (pL->header.frame_id == "-1") 
            {
              ptr->header = pL->header;
            }
            else 
            {
              ptr->header = pR->header;
            }
            
            _node->Publish(ptr);

            std::chrono::steady_clock::time_point end_point = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_point - start_point);
            std::cout << "[" <<  _node->get_name() << "]" << ": spend time: " << duration.count() << "ms" << std::endl;
            test_time = true;
            // RCLCPP_INFO(rclcpp::get_logger("laser_line_reconstruct")," publish line pointcloud, header.frame_id = -1");//todo

               


        }
      } 
      else
      {
        // start_point = std::chrono::steady_clock::now();
        _con.wait(lk);
      }
    }
  }

  void _PopFront(
    LineCenter::UniquePtr & pL,
    LineCenter::UniquePtr & pR)
  {
    pL = std::move(_deqL.front());
    pR = std::move(_deqR.front());
    _deqL.pop_front();
    _deqR.pop_front();
    // if(std::stoi(pL->header.frame_id)%10 == 0)
    // {
      // std::cout<<"[laser_line_reconstruct] frame_id: "<< pL->header.frame_id<<std::endl;
    // }
    // if(pL->header.frame_id == "-1")
    // {
    //   std::cout<<"[laser_line_reconstruct] pl frame_id: "<< pL->header.frame_id<<std::endl;
    // }
    
  }

  bool _Pair(
    LineCenter::UniquePtr & pL,
    LineCenter::UniquePtr & pR)
  {
    if (_deqL.empty() || _deqR.empty()) {return false;}

    auto idL = std::stoi(_deqL.front()->header.frame_id);
    auto idR = std::stoi(_deqR.front()->header.frame_id);

    if (idL == idR)
    {
      // std::cout<<"_deqL.id: " << idL << " = _deqR.id: " << idR <<std::endl;
      //  cv::waitKey(5);
      _PopFront(pL, pR);
      return true;
    } 
    else if (idL > idR && idR > 0)
    {

        // std::cout<<"_deqL.id: " << idL << " > _deqR.id: " << idR <<std::endl;
        //  cv::waitKey(5);
        for (auto iter = _deqR.begin(); iter != _deqR.end(); ++iter)
        {
            auto idT = std::stoi((*iter)->header.frame_id);
            // std::cout<<"idT: " << idT << std::endl;
            // cv::waitKey(5);
            if(idT == -1)
            {
                  _deqR.erase(_deqR.begin(), iter);
                  return false;
            }
            if (idL == idT)
            {
                std::cout<<"_deqR.id:"<<(*iter)->header.frame_id<<std::endl;
                //  cv::waitKey(5);
                _deqR.erase(_deqR.begin(), iter);
                _PopFront(pL, pR);
                return true;
            } 
        }
    }
    else if (idR > idL && idL > 0)
    {
        std::cout<<"_deqR.id: " << idR << " > _deqL.id: " << idL <<std::endl;
        //  cv::waitKey(5);
        for (auto iter = _deqL.begin(); iter != _deqL.end(); ++iter)
        {
            auto idT = std::stoi((*iter)->header.frame_id);
            std::cout<<"idT: " << idT << std::endl;
            // cv::waitKey(5);
            if(idT == -1)
            {
                _deqL.erase(_deqL.begin(), iter);
                return false;
            }
            else if (idR == idT)
            {
                std::cout<<"_deqL.id:"<<(*iter)->header.frame_id<<std::endl;
                //  cv::waitKey(5);
                _deqL.erase(_deqL.begin(), iter);
                _PopFront(pL, pR);
                return true;
            } 
        }
    }
    else if(idR == -1 || idL == -1)
    {
        std::cout<<"idR == -1 || idL == -1, _deqL.clear(), _deqR.clear()"<<std::endl;
        //  cv::waitKey(5);
        _PopFront(pL, pR);
        _deqL.clear();
        _deqR.clear();
        return true;
    }

    return false;
    // else
    // {
    //     throw std::runtime_error("failed pair");
    // }

    // else if (idL > idR)
    // {

    //   std::cout<<"_deqL.id > _deqR.id" <<std::endl;
    //   for (auto iter = _deqR.begin(); iter != _deqR.end(); ++iter)
    //   {
    //     std::cout<<"_deqR current id: "<<(*iter) -> header.frame_id <<std::endl;
    //     if( (*iter) -> header.frame_id == "-1")
    //     {
    //       for (auto iter = _deqL.begin(); iter != _deqL.end(); ++iter)
    //       {
    //         if (idR == std::stoi((*iter)->header.frame_id))
    //         {
    //           std::cout<<"_deqL.id: "<<(*iter)->header.frame_id<<std::endl;
    //           _deqL.erase(_deqL.begin(), iter);
    //           _PopFront(pL, pR);
    //           return true;
    //         }
    //       }
    //       _deqL.clear();
    //       std::cout<<"_deqL.clear()"<<std::endl;
    //       return false;
    //     }
    //     else 
    //     {
    //       if (idL == std::stoi((*iter)->header.frame_id))
    //       {
    //         std::cout<<"_deqR.id:"<<(*iter)->header.frame_id<<std::endl;
    //         _deqR.erase(_deqR.begin(), iter);
    //         _PopFront(pL, pR);
    //         return true;
    //       }          
    //       // _deqR.clear();
    //       // std::cout<<"_deqR.clear()"<<std::endl;
    //       // return false; 
    //     }
    //   } 
    //   _deqR.clear();
    //   std::cout<<"_deqR.clear()"<<std::endl;
    //   return false; 

    // } 
    // else //idR > idL
    // {
    //   std::cout<<"_deqR.id > _deqL.id" <<std::endl;
    //   for (auto iter = _deqL.begin(); iter != _deqL.end(); ++iter)
    //   {
    //     std::cout<<"_deqL.id: "<<(*iter)->header.frame_id <<std::endl;
    //     if( (*iter)->header.frame_id == "-1")
    //     {
    //       for (auto iter = _deqR.begin(); iter != _deqR.end(); ++iter)
    //       {
    //         if (idL == std::stoi((*iter)->header.frame_id))
    //         {
    //           std::cout<<"_deqR.id: " << (*iter)->header.frame_id<<std::endl;
    //           _deqR.erase(_deqR.begin(), iter);
    //           _PopFront(pL, pR);
    //           return true;
    //         } 
    //       }
    //       _deqR.clear();
    //       std::cout<<"_deqR.clear()"<<std::endl;
    //       return false;
    //     }
    //     else
    //     {
    //       if (idR == std::stoi((*iter)->header.frame_id)) 
    //       {
    //         _deqL.erase(_deqL.begin(), iter);
    //         _PopFront(pL, pR);
    //         return true;              
    //       }

    //     }

    //   }
    //   _deqL.clear();
    //   std::cout<<"_deqL.clear()"<<std::endl;
    //   return false;

    // }
  }

  float _InterpolateX(const cv::Point2f & p0, const cv::Point2f & p1, float y)
  {
    return p0.x + (p1.x - p0.x) * (y - p0.y) / (p1.y - p0.y);
  }

  void _InitializeParameters()
  {
    _node->declare_parameter("frame_id");

    _node->declare_parameter("camera_matrix_l");
    _node->declare_parameter("camera_matrix_r");
    _node->declare_parameter("dist_coeffs_l");
    _node->declare_parameter("dist_coeffs_r");
    _node->declare_parameter("rect_l");
    _node->declare_parameter("rect_r");
    _node->declare_parameter("proj_l");
    _node->declare_parameter("proj_r");
    _node->declare_parameter("q");
  }

  void _UpdateParameters()
  {
    _node->get_parameter("frame_id", _frameID);
  }

private:
  std::string _frameID = "inclinometer";

  cv::Mat _c[2];  ///< Camera matrix
  cv::Mat _d[2];  ///< Distortion coefficients
  cv::Mat _r[2];  ///< Rectification transformation
  cv::Mat _p[2];  ///< New projection matrix
  cv::Mat _q;     ///< 4x4 floating-point transformation matrix

  mutable std::vector<cv::Point2f> _pL, _pR, _unpL, _unpR;
  mutable std::vector<cv::Point3f> _pnts;
  mutable std::vector<cv::Point2i> _uv;

  LaserLineReconstruct * _node;
  std::mutex _mutex;              ///< Mutex to protect shared storage
  std::condition_variable _con;   ///< Conditional variable rely on mutex
  std::deque<LineCenter::UniquePtr> _deqL;
  std::deque<LineCenter::UniquePtr> _deqR;
  std::thread _thread;
};

LaserLineReconstruct::LaserLineReconstruct(const rclcpp::NodeOptions & options)
: Node("laser_line_reconstruct_node", options)
{
  _pub = this->create_publisher<PointCloud2>(_pubName, 50);

  _impl = std::make_unique<_Impl>(this);

  _cbgL = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subL_opt = rclcpp::SubscriptionOptions();
  subL_opt.callback_group = _cbgL;
  _subL = this->create_subscription<LineCenter>(
    _subLName,
    50,
    [this](LineCenter::UniquePtr ptr)
    {
      try {
        _impl->PushBackL(std::move(ptr));
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Exception in subscription: %s", e.what());
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Exception in subscription: unknown");
      }
    },
    subL_opt
  );

  _cbgR = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subR_opt = rclcpp::SubscriptionOptions();
  subR_opt.callback_group = _cbgR;
  _subR = this->create_subscription<LineCenter>(
    _subRName,
    50,
    [this](LineCenter::UniquePtr ptr)
    {
      try {
        _impl->PushBackR(std::move(ptr));
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Exception in subscription: %s", e.what());
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Exception in subscription: unknown");
      }
    },
    subR_opt
  );

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

LaserLineReconstruct::~LaserLineReconstruct()
{
  _subL.reset();
  _subR.reset();
  _impl.reset();
  _pub.reset();

  RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
}

}  // namespace laser_line_reconstruct

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(laser_line_reconstruct::LaserLineReconstruct)
