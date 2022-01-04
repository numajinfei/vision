#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "shared_interfaces/msg/roll_pitch_yaw.hpp"

namespace inclinometer
{

class Inclinometer : public rclcpp::Node
{
public:
    explicit Inclinometer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~Inclinometer();

private:
    void _InitializeParameters();
    void _UpdateParameters();

private:
    int _hz = 10;
    std::string _frame = "map";
    std::string _child = "inclinometer";
    tf2_ros::TransformBroadcaster tf_broadcaster;

    const char* _pubName = "~/rpy";
    rclcpp::Publisher<shared_interfaces::msg::RollPitchYaw>::SharedPtr _pub;

    std::thread _thread;
};

}
