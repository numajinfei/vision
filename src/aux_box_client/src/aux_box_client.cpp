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

#include "aux_box_client/aux_box_client.hpp"
#include "tcp/tcp.hpp"

#include <memory>
#include <thread>
#include <chrono>
#include <iostream>

namespace aux_box_client
{

#pragma pack(1)
typedef struct _s_att_angle_t
{
    float x;
    float y;
}s_att_angle_t;

typedef struct _s_laser_range_t
{
    float d;
}s_laser_range_t;

struct ComMessage
{
    char op;
    union
    {
        s_att_angle_t angle;
        s_laser_range_t range;
    }ucontent;
};
#pragma pack()

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

class Aux_box_client::_Impl
{
public:
  explicit _Impl(Aux_box_client * ptr, std::string& cli_ip, int& port) : _node(ptr), _cli(cli_ip, port)
  {
    try
    {	    
      std::cout << "ip: " << cli_ip << " port: " << port << std::endl;
      _cli.setMessageCallback(std::bind(&Aux_box_client::_Impl::_ParseAndPub, this, std::placeholders::_1, std::placeholders::_2));

      // _cli.setMessageCallback([this](char *buf, int len){
      //   _Parse(buf, len);
      // });

      _cli.Create();

      // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      // std::cout << "try to send data to Server...\n";
      // char buf[64] = {0};
      // strcpy(buf, "data from Client....");
      // buf[strlen("data from Client....")] = '\0';
      // _cli.Write(buf, strlen(buf));    
    }
    catch(const std::exception &e)
    {
      RCLCPP_ERROR(_node->get_logger(), "Exception in client create: %s", e.what());
    }
    catch(...)
    {
      RCLCPP_ERROR(_node->get_logger(), "Exception in client create: unkown");
    }
  }

  ~_Impl()
  {

  }

  void _ParseAndPub(char *buf, int len)
  {
    _node->SetStatus(true);
    auto m = (struct ComMessage*)buf;
    std::cout << "m->op: " << m->op << std::endl;
    if(len)
    {
      // char *a = "Read Att from server";
      // char *l = "Read LR from server";

      if(m->op == 'A')
      {
        // std::cout << "read inclinometer's data...\n";

        // _cli.Write(a, 18);
        // // auto s = std_msgs::msg::String();
        // // s.data = "ros2-->Read Att from server";
        // // _pub->publish(s);


        // shared_interfaces::msg::AuxBox x;
        // x.type = "Inclinometer";
        // x.v1 = m->ucontent.angle.x;
        // x.v2 = m->ucontent.angle.y;

        _node->_PubRollPitchYaw(m->ucontent.angle.y, m->ucontent.angle.x, 0.);
        _node->_PubInclinometer(m->ucontent.angle.x, m->ucontent.angle.y);

        //RCLCPP_INFO(_node->get_logger(), "read Inclinometer's angle...[%.4f],[%.4f]",m->ucontent.angle.x, m->ucontent.angle.y);
      }
      else if(m->op == 'L')
      {
        // std::cout << "read LR's data...\n";
        // _cli.Write(l, 17);

        // shared_interfaces::msg::AuxBox x;
        // x.type = "LaserRange";
        // x.v1 = m->ucontent.range.d;
        // x.v2 = .0;

        _node->_PubLaserRanger(m->ucontent.range.d);

        RCLCPP_INFO(_node->get_logger(), "read LaserRange's data...[%.2f]", m->ucontent.range.d);
      }
    }
    
  }

  void _TcpWrite(ComMessage& m)
  {
    // std::cout << "message: " << m.ucontent.range.d << std::endl;
    // std::cout << "size of m: " << sizeof(m) << std::endl;
    _cli.Write((char *)&m, sizeof(m));
    
    // std::cout << "write" << std::endl;
  }

  bool _isTcpConnected()
  {
    return _cli.isConnected();
  }

private:
  Aux_box_client * _node;
  net::Client _cli;
  // ComMessage _m;
  // rclcpp::Publisher<shared_interfaces::msg::AuxBox>::SharedPtr _pub;
};




/**** Aux_box_client ****/
Aux_box_client::Aux_box_client(const rclcpp::NodeOptions & options) : Node("aux_box_client_node", options)
{
  _initThread = std::thread(&Aux_box_client::_Init, this);
}

Aux_box_client::~Aux_box_client() try
{
  _initThread.join();

  _impl.reset();
  _pubLaserRanger.reset();
  _pubRollPitchYaw.reset();
  _pubInclinometer.reset();
  _srvGrabLaserRanger.reset();
  _srvGrabRollPitchYaw.reset();

  RCLCPP_INFO(this->get_logger(), "Aux_box_client: Destroyed successfully");
}
catch (const std::exception & e) 
{
    RCLCPP_ERROR(this->get_logger(), "Exception in destruction: %s", e.what());
    rclcpp::shutdown();
} 
catch (...) 
{
    RCLCPP_ERROR(this->get_logger(), "Exception in destruction: unknown");
    rclcpp::shutdown();
}


void Aux_box_client::Work() try
{
  int times = 0; 
  while(true)
  {
    // std::cout << "work" << std::endl;
    if(_impl->_isTcpConnected())
    {            
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      //std::cout << "publish a request to aux_box" << std::endl; 
      ComMessage m;
      m.op = 'A';
      m.ucontent.angle.x = .0;
      m.ucontent.angle.y = .0;

      _impl->_TcpWrite(m);         
    }

    if((++times) >= 20)//timebase 100ms
    {
      times = 0;
      std::cout << "_impl->_isTcpConnected(): " << _impl->_isTcpConnected() << std::endl;
      std::cout << "[Aux].(Work)--> linked: " << _impl->_isTcpConnected()  << " status: " << status << std::endl;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));    
  }
}
catch (const std::exception & e) 
{
    RCLCPP_ERROR(this->get_logger(), "Exception in Work(): %s", e.what());
    rclcpp::shutdown();
} 
catch (...) 
{
    RCLCPP_ERROR(this->get_logger(), "Exception in Work(): unknown");
    rclcpp::shutdown();
}

void Aux_box_client::_Init() try
{
    _status = -1;

    _InitializeParameters();

    _UpdateParameters();

    _pubLaserRanger = this->create_publisher<shared_interfaces::msg::LaserRanger>(_pubLaserRangerName, 10);

    _pubRollPitchYaw = this->create_publisher<shared_interfaces::msg::RollPitchYaw>(_pubRollPitchYawName, 10);

    _pubInclinometer = this->create_publisher<shared_interfaces::msg::Inclinometer>(_pubInclinometerName, 10);

    _srvGrabLaserRanger = this->create_service<shared_interfaces::srv::TriggerOp>(
      _srvGrabLaserRangerName, 
      std::bind(&Aux_box_client::_SrvGrabLaserRanger, this, std::placeholders::_1, std::placeholders::_2));

    _srvGrabRollPitchYaw = this->create_service<shared_interfaces::srv::TriggerOp>(
      _srvGrabRollPitchYawName,
      std::bind(&Aux_box_client::_SrvGrabRollPitchYaw, this, std::placeholders::_1, std::placeholders::_2));


    std::this_thread::sleep_for(std::chrono::milliseconds(3000)); //sleep time to make sure client connect to Server successfully.
    _impl = std::make_unique<_Impl>(this, _ip, _port);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); //sleep time to make sure client connect to Server successfully.
    std::thread th(&Aux_box_client::Work,this);
    th.detach();

    _status = 0;

    RCLCPP_INFO(this->get_logger(), "Aux_box_client: Initialized successfully");
}
catch (const std::exception & e) 
{
    RCLCPP_ERROR(this->get_logger(), "Exception in initializer: %s", e.what());
    rclcpp::shutdown();
} 
catch (...) 
{
    RCLCPP_ERROR(this->get_logger(), "Exception in initializer: unknown");
    rclcpp::shutdown();
}

void Aux_box_client::_InitializeParameters()
{
  this->declare_parameter<int>("port",6001);
  this->declare_parameter<std::string>("ip","192.168.30.3");
}

void Aux_box_client::_UpdateParameters()
{
  this->get_parameter("ip", _ip);
  this->get_parameter("port", _port);
}

void Aux_box_client::_SrvGrabLaserRanger(
  const std::shared_ptr<shared_interfaces::srv::TriggerOp::Request>,
  std::shared_ptr<shared_interfaces::srv::TriggerOp::Response> response)
{
  if(_status < 0)
  {
    RCLCPP_INFO(this->get_logger(), "%s initialize and receive a service call: grab laser.", GetLocalTime());
    return;
  }
  response->success = false;
  response->message = "Fail: grab laser ranger";

  std::cout << "\n\nsrv grab laser range" << std::endl;
  ComMessage m;
  m.op = 'L';
  m.ucontent.range.d = -1.;

  _impl->_TcpWrite(m);
  
  std::cout << "\n\n_SrvGrabLaserRanger(): laser range: " << m.ucontent.range.d << std::endl;

  response->success = true;
  response->message = "Success: grab laser ranger";

}

void Aux_box_client::_SrvGrabRollPitchYaw(
  const std::shared_ptr<shared_interfaces::srv::TriggerOp::Request>,
  std::shared_ptr<shared_interfaces::srv::TriggerOp::Response> response)
{
  if(_status < 0)
  {
    RCLCPP_INFO(this->get_logger(), "%s initialize and receive a service call: grab RPY.", GetLocalTime());
    return;
  }

  response->success = false;
  response->message = "Fail: grab roll_pitch_yaw";

  ComMessage m;
  m.op = 'A';
  m.ucontent.angle.x = .0;
  m.ucontent.angle.y = .0;

  _impl->_TcpWrite(m);

  response->success = true;
  response->message = "Success: grab roll_pitch_yaw";

}

void Aux_box_client::_PubLaserRanger(const double& laserRanger)
{
  if(_status < 0)
  {
    RCLCPP_INFO(this->get_logger(), "%s initialize and publish a laser.", GetLocalTime());
    return;
  }
  auto laserRangerPtr = std::make_unique<shared_interfaces::msg::LaserRanger>();
  laserRangerPtr->distance = laserRanger;

  _pubLaserRanger->publish(std::move(laserRangerPtr));
}

void Aux_box_client::_PubRollPitchYaw(const double& roll, const double& pitch, const double& yaw)
{
  if(_status < 0)
  {
    RCLCPP_INFO(this->get_logger(), "%s initialize and publish a RPY.", GetLocalTime());
    return;
  }

  auto rollPitchYawPtr = std::make_unique<shared_interfaces::msg::RollPitchYaw>();
  rollPitchYawPtr->roll = roll;
  rollPitchYawPtr->pitch = pitch;
  rollPitchYawPtr->yaw = yaw;

  _pubRollPitchYaw->publish(std::move(rollPitchYawPtr));
}

void Aux_box_client::_PubInclinometer(const float& angleX, const float& angleY)
{
  if(_status < 0)
  {
    RCLCPP_INFO(this->get_logger(), "%s initialize and publish a RPY.", GetLocalTime());
    return;
  }

  auto inclinometerPtr = std::make_unique<shared_interfaces::msg::Inclinometer>();
  inclinometerPtr->angle_x = angleX;
  inclinometerPtr->angle_y = angleY;

  _pubInclinometer->publish(std::move(inclinometerPtr));
}

}  // namespace aux_box_client


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(aux_box_client::Aux_box_client)
