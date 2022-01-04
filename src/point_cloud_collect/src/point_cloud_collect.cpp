#include "point_cloud_collect/point_cloud_collect.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include <deque>

namespace point_cloud_collect
{

class PointCloudCollect::_Impl
{
public:
    explicit _Impl(PointCloudCollect* ptr) : _node(ptr)
    {
        _InitializeParameters();

        _UpdateParameters();

        _InitialCloud();

        _thread = std::thread(&PointCloudCollect::_Impl::_Worker, this);
    }

    ~_Impl()
    {
        _con.notify_all();
        _thread.join();
    }

    void PushBack(sensor_msgs::msg::PointCloud2::UniquePtr& ptr)
    {
        std::unique_lock<std::mutex> lk(_mutex);
        _deq.emplace_back(std::move(ptr));
        lk.unlock();
        _con.notify_all();
    }

private:
    void _Worker()
    {
        while(rclcpp::ok())
        {
            std::unique_lock<std::mutex> lk(_mutex);
            if(_deq.empty() == false)
            {
                auto ptr = std::move(_deq.front());
                _deq.pop_front();
                lk.unlock();
                if(ptr->header.frame_id == "-1")
                    _Publish();
                else
                    _Collect(ptr);
            }
            else
                _con.wait(lk);
        }
    }

    void _Collect(sensor_msgs::msg::PointCloud2::UniquePtr& ptr)
    {
        if(ptr->width != 0)
        {
            pcl::PointCloud<pcl::PointXYZI> cloud;
            pcl::moveFromROSMsg(*ptr, cloud);
            *_cloud += cloud;
        }
    }

    void _Publish()
    {
        auto ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();

        if(_cloud->width != 0)
        {
            for(auto& p : *_cloud)
                p.intensity = 0;
            pcl::toROSMsg(*_cloud, *ptr);
            ptr->header.stamp = _node->now();
            ptr->header.frame_id = _frameID;//TODO
            _node->_pub->publish(std::move(ptr));
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "point_cloud_collect collect nothing");
        }

        _InitialCloud();
    }

    void _InitialCloud()
    {
        _cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        _cloud->height = 1;
        _cloud->width = 0;
    }

    void _InitializeParameters()
    {
        _node->declare_parameter("frame_id");
    }

    void _UpdateParameters()
    {
        _node->get_parameter("frame_id", _frameID);
    }

private:
    std::string _frameID = "inclinometer";

    PointCloudCollect* _node;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud;
    std::mutex _mutex;              ///< Mutex to protect shared storage
    std::condition_variable _con;   ///< Conditional variable rely on mutex
    std::deque<sensor_msgs::msg::PointCloud2::UniquePtr> _deq;
    std::thread _thread;
};

PointCloudCollect::PointCloudCollect(const rclcpp::NodeOptions& options) : Node("point_cloud_collect_node", options)
{
    _pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(_pubName, 1);

    _impl = std::make_unique<_Impl>(this);

    _sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(_subName, 50, std::bind(&PointCloudCollect::_Sub, this, std::placeholders::_1));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "point_cloud_collect initialized successfully");
}

PointCloudCollect::~PointCloudCollect()
{
    _sub.reset();
    _impl.reset();
    _pub.reset();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "point_cloud_collect destroyed successfully");
}

void PointCloudCollect::_Sub(sensor_msgs::msg::PointCloud2::UniquePtr ptr) try
{
    _impl->PushBack(ptr);
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in point_cloud_collect subscription: %s", e.what());
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in point_cloud_collect subscription: unknown");
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(point_cloud_collect::PointCloudCollect)

