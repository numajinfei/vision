#include "camera_galaxy/camera_galaxy.hpp"

#include <exception>

extern "C"
{
    #include "GxIAPI.h"
}

using namespace std::chrono_literals;

namespace camera_galaxy
{

void GX_STDC _OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame) try
{
    if(pFrame->status != GX_FRAME_STATUS_SUCCESS)
        throw std::runtime_error("Callback pFrame status error");

    auto node = static_cast<CameraGalaxy*>(pFrame->pUserParam);

    auto ptrL = std::make_unique<sensor_msgs::msg::Image>();
    auto ptrR = std::make_unique<sensor_msgs::msg::Image>();
    ptrL->header.stamp = ptrR->header.stamp = node->now();
    ptrL->header.frame_id = ptrR->header.frame_id = std::to_string(pFrame->nFrameID);
    auto h = ptrL->height = ptrR->height = pFrame->nHeight;
    auto w = ptrL->width = ptrR->width = pFrame->nWidth / 2;
    ptrL->encoding = ptrR->encoding = "mono8";
    ptrL->is_bigendian = ptrR->is_bigendian = false;
    ptrL->step = ptrR->step = pFrame->nWidth / 2;
    ptrL->data.resize(h * w); ptrR->data.resize(h * w);
    for(decltype(h) r = 0; r < h; ++r)
    {
        auto headI = (char*) (pFrame->pImgBuf) + w * r * 2;
        auto headL = (char*) (ptrL->data.data()) + w * r;
        auto headR = (char*) (ptrR->data.data()) + w * r;
        memcpy(headL, headI, w);
        memcpy(headR, headI + w, w);
    }

    node->PublishL(ptrL);
    node->PublishR(ptrR);
}
catch(const std::exception& e)
{
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in camera callback: %s", e.what());
}
catch(...)
{
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in camera callback: unknown");
}

class CameraGalaxy::_Impl
{
public:
    explicit _Impl(CameraGalaxy* ptr) : _node(ptr)
    {
        _InitializeParameters();

        if(GX_STATUS_SUCCESS != GXInitLib())
            throw std::runtime_error("Can not initialize Daheng galaxy libraries");

        while(true)
        {
            if(!rclcpp::ok())
            {
                GXCloseLib();
                throw std::runtime_error("Interrupted!");
            }

            uint32_t nDeviceNum = 0;
            if(GX_STATUS_SUCCESS == GXUpdateDeviceList(&nDeviceNum, 200) && nDeviceNum > 0)
                break;
        }

        GX_OPEN_PARAM stOpenParam;
        stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
        stOpenParam.openMode = GX_OPEN_INDEX;
        char content[] = "1";
        stOpenParam.pszContent = content;

        if(GX_STATUS_SUCCESS != GXOpenDevice(&stOpenParam, &_handle))
        {
            GXCloseLib();
            throw std::runtime_error("Can not open Daheng galaxy camera");
        }

        _WarmUp();

        if(GX_STATUS_SUCCESS != GXRegisterCaptureCallback(_handle, _node, _OnFrameCallbackFun))
        {
            GXCloseDevice(_handle);
            GXCloseLib();
            throw std::runtime_error("Can not register capture callback");
        }
    }

    ~_Impl()
    {
        GXStreamOff(_handle);
        GXUnregisterCaptureCallback(_handle);
        GXCloseDevice(_handle);
        GXCloseLib();
    }

    void Start()
    {
        _UpdateParameters();

        _ApplyParameters();

        _StreamOn();
    }

    void Stop()
    {
        _StreamOff();
    }

private:
    void _InitializeParameters()
    {
        _node->declare_parameter("acqusition_buffer_number");
        _node->declare_parameter("acqusition_frame_rate");
        _node->declare_parameter("exposure_time");
    }

    void _UpdateParameters()
    {
        _node->get_parameter("acqusition_buffer_number", _acqusitionBufferNumber);
        _node->get_parameter("acqusition_frame_rate", _acqusitionFrameRate);
        _node->get_parameter("exposure_time", _exposureTime);
    }

    void _ApplyParameters()
    {
        if(GX_STATUS_SUCCESS != GXSetAcqusitionBufferNumber(_handle, _acqusitionBufferNumber))
            throw std::runtime_error("Can not set paramter: acqusition buffer number");
        if(GX_STATUS_SUCCESS != GXSetFloat(_handle, GX_FLOAT_ACQUISITION_FRAME_RATE, _acqusitionFrameRate))
            throw std::runtime_error("Can not set paramter: acqusition frame rate");
        if(GX_STATUS_SUCCESS != GXSetFloat(_handle, GX_FLOAT_EXPOSURE_TIME, _exposureTime))
            throw std::runtime_error("Can not set paramter: exposure time");
    }

    void _StreamOn()
    {
        if(GX_STATUS_SUCCESS != GXStreamOn(_handle))
            throw std::runtime_error("Error: stream on");
    }

    void _StreamOff()
    {
        if(GX_STATUS_SUCCESS != GXStreamOff(_handle))
            throw std::runtime_error("Error: stream off");
    }

    void _WarmUp()
    {
        _UpdateParameters();

        _ApplyParameters();

        _StreamOn();

        for(int i = 0; i < 25; ++i)
        {
            if(!rclcpp::ok())
            {
                GXStreamOff(_handle);
                GXCloseLib();
                throw std::runtime_error("Interrupted!");
            }
            std::this_thread::sleep_for(200ms);
        }

        _StreamOff();
    }

private:
    int _acqusitionBufferNumber = 300;  ///< Acqusition buffer number
    double _acqusitionFrameRate = 85.;  ///< Acqusition frame rate
    double _exposureTime        = 200.; ///< Exposure time: us
    CameraGalaxy* _node;
    GX_DEV_HANDLE _handle = NULL;
};

CameraGalaxy::CameraGalaxy(const rclcpp::NodeOptions& options) : Node("camera_galaxy_node", options)
{
    _init = std::thread(&CameraGalaxy::_Init, this);
}

CameraGalaxy::~CameraGalaxy()
{
    _init.join();

    _srvStart.reset();
    _srvStop.reset();
    _impl.reset();
    _pubR.reset();
    _pubL.reset();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "camera_galaxy destroyed successfully");
}

void CameraGalaxy::_Init() try
{
    _pubL = this->create_publisher<sensor_msgs::msg::Image>(_pubLName, 50);
    _pubR = this->create_publisher<sensor_msgs::msg::Image>(_pubRName, 50);

    _impl = std::make_unique<_Impl>(this);

    _srvStop = this->create_service<std_srvs::srv::Trigger>(_srvStopName, std::bind(&CameraGalaxy::_Stop, this, std::placeholders::_1, std::placeholders::_2));

    _srvStart = this->create_service<std_srvs::srv::Trigger>(_srvStartName, std::bind(&CameraGalaxy::_Start, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "camera_galaxy initialized successfully");
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_galaxy initializer: %s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_galaxy initializer: unknown");
    rclcpp::shutdown();
}

void CameraGalaxy::_Start(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) try
{
    response->success = false;
    response->message = "Fail: camera start";

    _impl->Start();

    response->success = true;
    response->message = "Success: camera start";
}
catch(const std::exception& e)
{
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in camera_galaxy service start: %s", e.what());
}
catch(...)
{
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in camera_galaxy service start: unknown");
}

void CameraGalaxy::_Stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) try
{
    response->success = false;
    response->message = "Fail: camera stop";

    _impl->Stop();

    auto ptrL = std::make_unique<sensor_msgs::msg::Image>();
    auto ptrR = std::make_unique<sensor_msgs::msg::Image>();
    ptrL->header.stamp = ptrR->header.stamp = now();
    ptrL->header.frame_id = ptrR->header.frame_id = "-1";
    PublishL(ptrL);
    PublishR(ptrR);
    response->success = true;
    response->message = "Success: camera stop";
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_galaxy service stop: %s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_galaxy service stop: unknown");
    rclcpp::shutdown();
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(camera_galaxy::CameraGalaxy)
