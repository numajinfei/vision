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

#ifndef AUX_BOX_CLIENT__AUX_BOX_CLIENT_HPP_
#define AUX_BOX_CLIENT__AUX_BOX_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
// #include "shared_interfaces/msg/aux_box.hpp"
#include "shared_interfaces/msg/laser_ranger.hpp"
#include "shared_interfaces/msg/roll_pitch_yaw.hpp"
#include "shared_interfaces/srv/trigger_op.hpp"
#include "shared_interfaces/msg/inclinometer.hpp"

namespace aux_box_client
{

class Aux_box_client : public rclcpp::Node
{
public:
  explicit Aux_box_client(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~Aux_box_client();

private:
  void _Init();
  void _InitializeParameters();
  void _UpdateParameters();
  // void _Sub(std_msgs::msg::String::UniquePtr ptr);  
  void _SrvGrabLaserRanger(
    const std::shared_ptr<shared_interfaces::srv::TriggerOp::Request>,
    std::shared_ptr<shared_interfaces::srv::TriggerOp::Response> response);
  void _SrvGrabRollPitchYaw(
    const std::shared_ptr<shared_interfaces::srv::TriggerOp::Request>,
    std::shared_ptr<shared_interfaces::srv::TriggerOp::Response> response);
  void _PubLaserRanger(const double& laserRanger);
  void _PubRollPitchYaw(const double& roll, const double& pitch, const double& yaw);
  void _PubInclinometer(const float& angleX, const float& angleY);
  void Work();

private:
  int _status;
  
  class _Impl;
  std::unique_ptr<_Impl> _impl;

  const char* _pubLaserRangerName = "~/laserRanger";  
  rclcpp::Publisher<shared_interfaces::msg::LaserRanger>::SharedPtr _pubLaserRanger;

  const char* _pubRollPitchYawName = "~/rollPitchYaw";  
  rclcpp::Publisher<shared_interfaces::msg::RollPitchYaw>::SharedPtr _pubRollPitchYaw;

  const char* _pubInclinometerName = "~/inclinometer";  
  rclcpp::Publisher<shared_interfaces::msg::Inclinometer>::SharedPtr _pubInclinometer;

  const char* _srvGrabLaserRangerName = "~/grabLaserRanger";
  rclcpp::Service<shared_interfaces::srv::TriggerOp>::SharedPtr _srvGrabLaserRanger;

  const char* _srvGrabRollPitchYawName = "~/grabRollPitchYaw";
  rclcpp::Service<shared_interfaces::srv::TriggerOp>::SharedPtr _srvGrabRollPitchYaw; 

  // const char * _subStringName = "~/subString";  
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subString;

  std::thread _initThread;

private:
  int _port;
  std::string _ip;

  bool status;

  inline bool GetStatus()
  {
    return status;
  }

  inline void SetStatus(const bool& _status)
  {
    status = _status;
    //std::cout << "status: " << status <<std::endl;
  }
};

}  // namespace aux_box_client

#endif  // AUX_BOX_CLIENT__AUX_BOX_CLIENT_HPP_
