#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "shared_interfaces/msg/float64_array.hpp"

namespace script_json
{

class ScriptJson : public rclcpp::Node
{
public:
    explicit ScriptJson(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ScriptJson();

private:
    enum STATUS {STATUS_READY, STATUS_BUSY, STATUS_ERROR};
    void _Init();
    bool _IsNodesReady();
    bool _IsServicesReady();
    void _Execute();
    void _InitializeParameters();
    void _UpdateParameters();
    void _Sub(std_msgs::msg::String::UniquePtr ptr);
    void _SubResult(shared_interfaces::msg::Float64Array::UniquePtr ptr);
    void _PublishStatus(STATUS);
    void _PublishResult(int code, const std::vector<double>& result = {});

private:
    int _status = -1;
    int _id;
    std::string _param;
    std::vector<std::string> _requiredNodes;
    std::map<std::string, std::vector<std::string>> _requiredServices;
    std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> _map;
    std::unique_ptr<rclcpp::AsyncParametersClient> _paramClient;

    const char* _pubName = "~/response";
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;

    const char* _subName = "~/request";
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;

    const char* _subResultName = "~/result";
    rclcpp::Subscription<shared_interfaces::msg::Float64Array>::SharedPtr _subResult;

    std::thread _init;

    std::future<void> _future;
};

}

