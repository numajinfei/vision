#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "shared_interfaces/msg/float64_array.hpp"
#include "shared_interfaces/msg/roll_pitch_yaw.hpp"

namespace point_cloud_analyse
{

class PointCloudAnalyse : public rclcpp::Node
{
public:
    explicit PointCloudAnalyse(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PointCloudAnalyse();

private:
    void _Init();
    void _InitializeParameters();
    void _UpdateParameters();
    void _Sub(sensor_msgs::msg::PointCloud2::UniquePtr ptr);//TODO

private:
    std::string _frameID = "inclinometer";
    int _mode = 0;
    double _roll = 0., _pitch = 0., _yaw = 0.;

    //const char* _pubName = "~/pub";//TODO
    //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;

    const char* _pubResultName = "~/result";
    rclcpp::Publisher<shared_interfaces::msg::Float64Array>::SharedPtr _pubResult;

    class _Impl;
    std::unique_ptr<_Impl> _impl;

    const char* _subName = "~/points";//TODO
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub;

    const char* _subRpyName = "~/rpy";
    rclcpp::Subscription<shared_interfaces::msg::RollPitchYaw>::SharedPtr _subRpy;

    std::thread _init;
};

}

