#pragma once
#ifndef _PHOXI_CONTROL_H
#define _PHOXI_CONTROL_H



#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <thread>
#include <atomic>
#include <memory>
#include <utility>

#if defined(_WIN32)
    #include <windows.h>
#elif defined (__linux__)
    #include <unistd.h>
#endif

#define PHO_IGNORE_CV_VERSION_RESTRICTION
#define PHOXI_OPENCV_SUPPORT
#define PHOXI_PCL_SUPPORT

#include "PhoXi.h"

#include "opencv2/core.hpp"

#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/point_cloud_op.hpp"
#include "shared_interfaces/msg/point_cloud_c.hpp"
#include "shared_interfaces/msg/image_c.hpp"
#include "shared_interfaces/srv/trigger_op.hpp"

// #include "yaml-cpp/yaml.h"

#if defined(_WIN32)
    #define LOCAL_CROSS_SLEEP(Millis) Sleep(Millis)
    #define DELIMITER "\\"
#elif defined (__linux__) || defined(__APPLE__)
    #define DELIMITER "/"
#endif

namespace phoxi_control 
{
//The whole api is in namespace pho (Photoneo) :: api

class PhoXiControl : public rclcpp::Node
{
    public:
        explicit PhoXiControl(const rclcpp::NodeOptions& option = rclcpp::NodeOptions());
        ~PhoXiControl();

    private:
        void _Init();
        void _InitializeParameters();
        void _UpdateParameters();

        void _SrvStart(
            const std::shared_ptr<shared_interfaces::srv::TriggerOp::Request>, 
            std::shared_ptr<shared_interfaces::srv::TriggerOp::Response> response);

        void _SrvStop(
            const std::shared_ptr<shared_interfaces::srv::TriggerOp::Request>, 
            std::shared_ptr<shared_interfaces::srv::TriggerOp::Response> response);
        
        void _SrvGrabPointCloud(
            const std::shared_ptr<shared_interfaces::srv::TriggerOp::Request> request, 
            std::shared_ptr<shared_interfaces::srv::TriggerOp::Response> response);

        void _SrvGrabImage(
            const std::shared_ptr<shared_interfaces::srv::TriggerOp::Request> request,
            std::shared_ptr<shared_interfaces::srv::TriggerOp::Response> reponse);
        
        void _PubPointCloud(
            const pho::api::PFrame &frame, 
            const long& id,
            const std::string& engineering, 
            const std::string& option,
            const float& angle,
            const int& frameID);

        void _PubImage(
            const cv::Mat& image,
            const long& id,
            const std::string& engineering,
            const std::string& option,
            const float& angle,
            const int& feameID);
        void _PubStatus(const std::string& status);

    private:
        int _status;
        
        class _Impl;
        std::unique_ptr<_Impl> _impl;

        const char* _srvGrabPointCloudName = "~/grab_point_cloud";
        rclcpp::Service<shared_interfaces::srv::TriggerOp>::SharedPtr _srvGrabPointCloud;

        const char* _srvGrabImageName = "~/grab_image";
        rclcpp::Service<shared_interfaces::srv::TriggerOp>::SharedPtr _srvGrabImage;

        const char* _srvStartName = "~/start";
        rclcpp::Service<shared_interfaces::srv::TriggerOp>::SharedPtr _srvStart;

        const char* _srvStopName = "~/stop";
        rclcpp::Service<shared_interfaces::srv::TriggerOp>::SharedPtr _srvStop;

        const char* _pubPointCloudName = "~/pointcloud";
        rclcpp::Publisher<shared_interfaces::msg::PointCloudC>::SharedPtr _pubPointCloud;

        const char* _pubImageEmbeddedPartsName = "~/image_embedded_parts";
        rclcpp::Publisher<shared_interfaces::msg::ImageC>::SharedPtr _pubImageEmbeddedParts;

        const char* _pubImageDefectDetectionName = "~/image_defect_detection";
        rclcpp::Publisher<shared_interfaces::msg::ImageC>::SharedPtr _pubImageDefectDetection;

        const char* _pubImageMortarJointName = "~/image_mortar_joint";
        rclcpp::Publisher<shared_interfaces::msg::ImageC>::SharedPtr _pubImageMortarJoint;

        const char* _pubStatusName = "~/status";
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pubStatus;

        std::thread _initThread;
};

}//namespace phoxi_control

#endif //_PHOXI_CONTROL_H