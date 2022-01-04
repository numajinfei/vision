#include "inclinometer/inclinometer.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

extern "C"
{
    #include "attitude_726.h"
}

namespace inclinometer
{

using namespace std::chrono_literals;

const double PI = std::acos(-1);
const double PI_2 = PI / 2.;

double Deg(double rad)
{
    return rad / PI * 180.;
}

double Rad(double deg)
{
    return deg / 180. * PI;
}

Inclinometer::Inclinometer(const rclcpp::NodeOptions& options) : Node("inclinometer_node", options), tf_broadcaster(this)
{
    _InitializeParameters();

    _UpdateParameters();
    
    _pub = this->create_publisher<shared_interfaces::msg::RollPitchYaw>(_pubName, 10);

    if(AttUartOpen() < 0)
        throw std::runtime_error("Can not open inclinometer");

    _thread = std::thread([this]()
    {
        while(rclcpp::ok())
        {
            float x = 0, y = 0;
            auto status = AttGetParam(&x, &y);
            if(status < 0)
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Error in inclinometer get param [%d]", status);
            }
            else
            {
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = this->now();
                transformStamped.header.frame_id = _frame;
                transformStamped.child_frame_id = _child;
                transformStamped.transform.translation.x = 0;
                transformStamped.transform.translation.y = 0;
                transformStamped.transform.translation.z = 0;

                tf2::Quaternion q;
                q.setRPY(Rad(x), Rad(-y), 0.);

                transformStamped.transform.rotation.x = q.x();
                transformStamped.transform.rotation.y = q.y();
                transformStamped.transform.rotation.z = q.z();
                transformStamped.transform.rotation.w = q.w();

                tf_broadcaster.sendTransform(transformStamped);
                shared_interfaces::msg::RollPitchYaw rpy;
                rpy.roll = Rad(x), rpy.pitch = Rad(-y), rpy.yaw = 0.;
                _pub->publish(rpy);
            }

            std::this_thread::sleep_for(std::chrono::microseconds(1000000 / _hz));
        }
    });

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inclinometer initialized successfully");
}

Inclinometer::~Inclinometer()
{
    _thread.join();
    AttUartClose();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inclinometer destroyed successfully");
}

void Inclinometer::_InitializeParameters()
{
    this->declare_parameter("hz");
    this->declare_parameter("frame");
    this->declare_parameter("child");
}

void Inclinometer::_UpdateParameters()
{
    this->get_parameter("hz", _hz);
    this->get_parameter("frame", _frame);
    this->get_parameter("child", _child);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(inclinometer::Inclinometer)
