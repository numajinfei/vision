#include "camera_spinnaker/camera_spinnaker.hpp"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std::chrono_literals;

namespace camera_spinnaker
{

// This class defines the properties, parameters, and the event handler itself. Take a
// moment to notice what parts of the class are mandatory, and what have been
// added for demonstration purposes. First, any class used to define image event handlers
// must inherit from ImageEventHandler. Second, the method signature of OnImageEvent()
// must also be consistent. Everything else - including the constructor,
// deconstructor, properties, body of OnImageEvent(), and other functions -
// is particular to the example.
class ImageEventHandlerImpl : public ImageEventHandler
{
public:
    // The constructor retrieves the serial number and initializes the image
    // counter to 0.
    ImageEventHandlerImpl(CameraSpinnaker* ptr) : _node(ptr)
    {
    }

    ~ImageEventHandlerImpl()
    {
    }

    // This method defines an image event. In it, the image that triggered the
    // event is converted and saved before incrementing the count. Please see
    // Acquisition_CSharp example for more in-depth comments on the acquisition
    // of images.
    void OnImageEvent(ImagePtr image) try
    {
        if(image->IsIncomplete())
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in camera_spinnaker callback: image is incomplete");
            return;
        }

        auto ptr = std::make_unique<sensor_msgs::msg::Image>();
        ptr->header.stamp = _node->now();
        ptr->header.frame_id = std::to_string(image->GetFrameID());
        ptr->height = image->GetHeight();
        ptr->width = image->GetWidth();
        ptr->encoding = "mono8";
        ptr->is_bigendian = false;
        ptr->step = image->GetStride();
        ptr->data.resize(image->GetBufferSize());

        memcpy((void*) ptr->data.data(), (void*) image->GetData(), image->GetBufferSize());

        _node->Publish(ptr);
    }
    catch(const Spinnaker::Exception& e)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in camera_spinnaker callback: %s", e.what());
    }
    catch(const std::exception& e)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in camera_spinnaker callback: %s", e.what());
    }
    catch(...)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in camera_spinnaker callback: unknown");
    }

private:
    CameraSpinnaker* _node;
};

class CameraSpinnaker::_Impl
{
public:
    _Impl(CameraSpinnaker* ptr) : _node(ptr)
    {
        system = System::GetInstance();

        while(true)
        {
            if(!rclcpp::ok())
            {
                system->ReleaseInstance();
                throw std::runtime_error("Interrupted!");
            }

            camList = system->GetCameras();
            if(camList.GetSize() > 0)
            {
                pCam = camList.GetByIndex(0);
                break;
            }
            else
            {
                camList.Clear();
                std::this_thread::sleep_for(200ms);
            }    
        }

        pCam->Init();

        imageEventHandler = new ImageEventHandlerImpl(_node);

        pCam->RegisterEventHandler(*imageEventHandler);
    }

    ~_Impl() try
    {
        if(pCam->IsStreaming())
            pCam->EndAcquisition();

        pCam->V3_3Enable = false;

        pCam->UnregisterEventHandler(*imageEventHandler);

        delete imageEventHandler;

        pCam->DeInit();

        pCam = nullptr;

        camList.Clear();

        system->ReleaseInstance();
    }
    catch(const Spinnaker::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_spinnaker impl destructor: %s", e.what());
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_spinnaker impl destructor: %s", e.what());
    }
    catch(...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_spinnaker impl destructor: unknown");
    }

    void Start()
    {
        pCam->V3_3Enable = true;//TODO
        pCam->BeginAcquisition();
    }

    void Stop()
    {
        pCam->EndAcquisition();
        pCam->V3_3Enable = false;
    }

private:
    CameraSpinnaker* _node;
    SystemPtr system;
    CameraList camList;
    CameraPtr pCam;
    ImageEventHandlerImpl* imageEventHandler;
};

CameraSpinnaker::CameraSpinnaker(const rclcpp::NodeOptions& options) : Node("camera_spinnaker_node", options)
{
    _init = std::thread(&CameraSpinnaker::_Init, this);
}

CameraSpinnaker::~CameraSpinnaker()
{
    _init.join();

    _srvStart.reset();
    _srvStop.reset();
    _impl.reset();
    _pub.reset();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "camera_spinnaker destroyed successfully");
}

void CameraSpinnaker::_Init() try
{
    _InitializeParameters();

    _UpdateParameters();

    _pub = this->create_publisher<sensor_msgs::msg::Image>(_pubName, 10);

    _impl = std::make_unique<_Impl>(this);

    _srvStop = this->create_service<std_srvs::srv::Trigger>(_srvStopName, std::bind(&CameraSpinnaker::_SrvStop, this, std::placeholders::_1, std::placeholders::_2));

    _srvStart = this->create_service<std_srvs::srv::Trigger>(_srvStartName, std::bind(&CameraSpinnaker::_SrvStart, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "camera_spinnaker initialized successfully");
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_spinnaker initializer: %s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_spinnaker initializer: unknown");
    rclcpp::shutdown();
}

void CameraSpinnaker::_SrvStart(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response) try
{
    response->success = false;
    response->message = "Fail: camera start";

    _impl->Start();

    response->success = true;
    response->message = "Success: camera start";
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_spinnaker service: %s", e.what());
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_spinnaker service: unknown");
}

void CameraSpinnaker::_SrvStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response) try
{
    response->success = false;
    response->message = "Fail: camera stop";

    _impl->Stop();

    response->success = true;
    response->message = "Success: camera stop";
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_spinnaker service: %s", e.what());
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_spinnaker service: unknown");
}

void CameraSpinnaker::_InitializeParameters()
{
    //this->declare_parameter("");
}

void CameraSpinnaker::_UpdateParameters()
{
    //this->get_parameter("", );
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(camera_spinnaker::CameraSpinnaker)

