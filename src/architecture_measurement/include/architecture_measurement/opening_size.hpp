
#pragma once
#ifndef _SURFACE_FLATNESS_NODE_H
#define _SURFACE_FLATNESS_NODE_H

// #include "architecture_measurement/impl/time.hpp"
#include "architecture_measurement/impl/inclinometer.hpp"
#include "architecture_measurement/impl/point_cloud_gui.hpp"
#include "architecture_measurement/impl/fundamental.hpp"
#include "architecture_measurement/impl/space_analytic_geometry.hpp"
#include "architecture_measurement/impl/cloud_pretreatment.hpp"
#include "architecture_measurement/impl/cloud_header_ex.hpp"

#include "architecture_measurement/opening_size/Global.h"
#include "architecture_measurement/opening_size/Utils.h"
#include "architecture_measurement/opening_size/FSMeasure.h"
#include "architecture_measurement/opening_size/FSMeasureImpl.h"

#include <string>
#include <vector>
#include <utility>
#include <thread>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <numeric>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/PointIndices.h"
#include "pcl/ModelCoefficients.h"

#include "nlohmann/json.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "shared_interfaces/msg/point_cloud_c.hpp"
#include "shared_interfaces/msg/inclinometer.hpp"
#include "shared_interfaces/msg/float64_array.hpp"
#include "shared_interfaces/msg/measurement_result.hpp"

#define AM_TEST

namespace am
{

using json = nlohmann::json;
using namespace std::chrono_literals;

class OpeningSize : public rclcpp::Node
{
    public:

        explicit OpeningSize(const rclcpp::NodeOptions& option = rclcpp::NodeOptions());

        ~OpeningSize();

    private:       

        void Init();

        void InitializeParameters();

        void UpdateParameters();

        void SubPointCloud(shared_interfaces::msg::PointCloudC::UniquePtr ptr);

        void SubInclinometer(shared_interfaces::msg::Inclinometer::UniquePtr ptr);

        void PubResult(const float& code, const std::vector<float>& result);

        void PubStatus(const std::string& status);

        std::vector<float> Calculate(
            const int& engineering,
            const int& object);

        int Convert(
            std::vector<pcl::PointCloud<pcl::PointXYZI>>& pointCloudVec);

    private:
        Inclinometer _inclinometer;
        std::vector<OpeningData> _openingDataVec;

        std::vector<std::string> _engineeringVec;
        std::vector<std::string> _optionVec;
        Eigen::Matrix3f _I_R_C;
        std::string _fileName;

        const char* _subPointCloudName = "~/pointcloud";
        rclcpp::Subscription<shared_interfaces::msg::PointCloudC>::SharedPtr _subPointCloud;

        const char* _subInclinometerName = "~/inclinometer";
        rclcpp::Subscription<shared_interfaces::msg::Inclinometer>::SharedPtr _subInclinometer;

        const char* _pubResultName = "~/result";
        rclcpp::Publisher<shared_interfaces::msg::MeasurementResult>::SharedPtr _pubResult;

        const char* _pubStatusName = "~/status";
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pubStatus;

        std::thread _initThread;

        int _status;
};

}// namespace am

#endif // _SURFACE_FLATNESS_NODE_H


