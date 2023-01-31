#pragma once
#ifndef _SURFACE_FLATNESS_H
#define _SURFACE_FLATNESS_H

#include "architecture_measurement/impl/fundamental.hpp"


#include <string>
#include <vector>
#include <utility>

#include "Eigen/Core"

//  #include "yaml-cpp/yaml.h"

#include "pcl/point_types.h"
#include "pcl/common/distances.h"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "shared_interfaces/msg/float64_array.hpp"
#include "shared_interfaces/msg/roll_pitch_yaw.hpp"
#include "shared_interfaces/msg/point_cloud_c.hpp"

namespace am
{

/** @class SurfaceFlatness
 * @brief 表面平整度节点类
 * 
 * 接收点云数据，识别平面，提取米字型靠尺，计算表面平整度
 */
class SurfaceFlatness : public rclcpp::Node
{
    public:
        /**
         * @brief 构造一个表面平整度对象
         * 
         * @param option ros2 节点选项
         */
        explicit SurfaceFlatness(const rclcpp::NodeOptions& option);

        /**
         * @brief 析构表面平整度对象
         * 
         */
        ~SurfaceFlatness();

    private:       
        /**
         * @brief 表面平整度对象初始化
         * 
         */
        void _Init();

        /**
         * @brief 声明ros2参数
         * 
         */
        void _InitializeParameters();

        /**
         * @brief 更新ros2参数
         * 
         */
        void _UpdateParameters();

        /**
         * @brief 订阅点云回调函数
         * 
         * @param ptr 点云数据指针
         */
        void _SubPointCloud(shared_interfaces::msg::PointCloudC::UniquePtr ptr);

        /**
         * @brief 订阅倾角仪回调函数
         * 
         * @param ptr 倾角仪数据指针
         */
        void _SubRpy(shared_interfaces::msg::RollPitchYaw::UniquePtr ptr);

        /**
         * @brief 测量结果发布函数
         * 
         * @param result 结果数据
         */
        void _PubResult(const std::vector<double>& result);

        /**
         * @brief 状态发布函数
         * 
         * @param status 状态信息
         */
        void _PubStatus(const std::string& status);

    private:
        double _roll, _pitch, _yaw;// 倾角仪数据

        std::vector<std::string> _engineeringVec;
        std::vector<std::string> _optionVec;

        int _status;

        class _Impl;// 嵌套类
        std::unique_ptr<_Impl> _impl;// 嵌套类指针

        const char* _subPointCloudName = "~/pointcloud";// 点云数据话题名称
        rclcpp::Subscription<shared_interfaces::msg::PointCloudC>::SharedPtr _subPointCloud;// 点云数据接收者指针

        const char* _subRpyName = "~/rollPitchYaw";// 倾角仪数据话题名称
        rclcpp::Subscription<shared_interfaces::msg::RollPitchYaw>::SharedPtr _subRpy;// 倾角仪数据接收者指针

        const char* _pubResultName = "~/result";// 结果话题名称
        rclcpp::Publisher<shared_interfaces::msg::Float64Array>::SharedPtr _pubResult;// 结果数据发布者指针

        const char* _pubStatusName = "~/status";// 状态话题名称
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pubStatus;// 状态信息发布者指针

        std::thread _initThread;// 线程对象
};

}// namespace am

#endif // _SURFACE_FLATNESS_H


