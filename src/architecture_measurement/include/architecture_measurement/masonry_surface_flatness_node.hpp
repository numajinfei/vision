#pragma once
#ifndef _MASONRY_SURFACE_FLATNESS_NODE_H
#define _MASONRY_SURFACE_FLATNESS_NODE_H

#include "architecture_measurement/masonry_surface_flatness.hpp"

#include <map>
#include <chrono>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "nlohmann/json.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "shared_interfaces/msg/point_cloud_c.hpp"
#include "shared_interfaces/msg/image_c.hpp"
#include "shared_interfaces/msg/inclinometer.hpp"
#include "shared_interfaces/msg/measurement_result.hpp"

namespace am
{

using json = nlohmann::json;
using namespace std::chrono_literals;

class MasonrySurfaceFlatnessNode : public rclcpp::Node
{
    public:

        MasonrySurfaceFlatnessNode(const rclcpp::NodeOptions& option = rclcpp::NodeOptions());

        ~MasonrySurfaceFlatnessNode();

        std::vector<float> Calculate();

    private:       

        void Init();

        void InitializeParameters();

        void UpdateParameters();

        void SubPointCloud(shared_interfaces::msg::PointCloudC::UniquePtr pointCloudCPtr);

        void SubImage(shared_interfaces::msg::ImageC::UniquePtr imageCPtr);

        void SubInclinometer(shared_interfaces::msg::Inclinometer::UniquePtr inclinometerPtr);

        void PubResult(const std::string& name, const float& code, const std::vector<float>& result);

        void PubStatus(const std::string& status);

    private:
        MasonrySurfaceFlatness _masonrySurfaceFlatness;
        CombinationDate _combinationDate;

        std::vector<std::string> _engineeringVec;
        std::vector<std::string> _optionVec;
        Eigen::Matrix3f _I_R_C;
        std::string _fileName;

        const char* _subPointCloudName = "~/pointcloud";// 点云数据话题名称
        rclcpp::Subscription<shared_interfaces::msg::PointCloudC>::SharedPtr _subPointCloud;// 点云数据接收者指针

        const char* _subImageName = "~/image";
        rclcpp::Subscription<shared_interfaces::msg::ImageC>::SharedPtr _subImage;

        const char* _subInclinometerName = "~/inclinometer";// 倾角仪数据话题名称
        rclcpp::Subscription<shared_interfaces::msg::Inclinometer>::SharedPtr _subInclinometer;// 倾角仪数据接收者指针

        const char* _pubResultName = "~/result";// 结果话题名称
        rclcpp::Publisher<shared_interfaces::msg::MeasurementResult>::SharedPtr _pubResult;// 结果数据发布者指针

        const char* _pubStatusName = "~/status";// 状态话题名称
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pubStatus;// 状态信息发布者指针

        std::thread _initThread;

        int _status;
};

}// namespace am

#endif // _MASONRY_SURFACE_FLATNESS_NODE_H


