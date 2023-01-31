#ifndef _FUNCTION_H
#define _FUNCTION_H



#include <algorithm>
#include <string>
#include <random>
#include <cstdlib>
#include <ctime>
#include <random>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "shared_interfaces/msg/float64_array.hpp"
#include "shared_interfaces/msg/defect.hpp"
#include "shared_interfaces/msg/embedded_parts.hpp"
#include "shared_interfaces/srv/trigger_op.hpp"
#include "shared_interfaces/msg/measurement_result.hpp"

#include "nlohmann/json.hpp"


namespace function
{

class Function : public rclcpp::Node
{
    public:
        explicit Function(const rclcpp::NodeOptions& options);
        ~Function();

    private:    
        void _Init();
        void _InitializeParameters();
        void _UpdateParameters();
        
        void _SubRequest(std_msgs::msg::String::UniquePtr ptr);

        void _SubResult(shared_interfaces::msg::Float64Array::UniquePtr ptr);
        void _SubResultEx(shared_interfaces::msg::MeasurementResult::UniquePtr ptr);
        void _SubResultDefect(shared_interfaces::msg::Defect::UniquePtr ptr);
        void _SubResultEmbeddedParts(shared_interfaces::msg::EmbeddedParts::UniquePtr ptr);

        void _PubResult(const int& code, const std::vector<double>& result = {});
        void _PubResult(const std::string& name, const float& code, const std::vector<float>& result = {});
        void _PubResult(const int& code, const int& value = 0, const std::string& imagePath = "");

        void _PubStatus(const int& code, const std::string& status = "");
        void _Execute();

    private:
        int _status;
        long _id;
        std::string _engineering;
        float _angle;
        std::string _value;
        int _frameID;

        class _Impl;
        std::unique_ptr<_Impl> _impl;

        const char* _pubResultName = "~/result_publisher";
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pubResult;

        const char* _pubStatusName = "~/status";
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pubStatus;

        const char* _subRequestName = "~/request";
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subRequest;

        const char* _subResultName = "~/result_subscription";
        rclcpp::Subscription<shared_interfaces::msg::Float64Array>::SharedPtr _subResult;

        const char* _subResultNameEx = "~/result_subscription_ex";
        rclcpp::Subscription<shared_interfaces::msg::MeasurementResult>::SharedPtr _subResultEx;

        const char* _subResultDefectName = "~/result_defect_subscription";
        rclcpp::Subscription<shared_interfaces::msg::Defect>::SharedPtr _subResultDefect;

        const char* _subResultEmbeddedPartsName = "~/result_mebedded_parts_subscription";
        rclcpp::Subscription<shared_interfaces::msg::EmbeddedParts>::SharedPtr _subResultEmbeddedParts;

        std::thread _initThread;

        std::future<void> _future;
};

}//namespace function

#endif //_FUNCTION_H

