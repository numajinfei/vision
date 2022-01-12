#include "camera_basler/camera_basler.hpp"

#include <exception>
#include <memory>
#include <fstream>
#include "opencv2/core.hpp" 
#include "opencv2/imgproc.hpp"
//#include "opencv2/highgui.hpp"
#include <opencv2/imgcodecs.hpp>

using namespace std::chrono_literals;

#include <pylon/PylonIncludes.h>
#include "camera_basler/ConfigurationEventPrinter.h"


// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using GenApi objects.
using namespace GenApi;

static camera_basler::CameraBasler * nodeptr = nullptr;


class ConfigBasler
{
public:
    explicit ConfigBasler(CInstantCamera& camera) : nodemap_(camera.GetNodeMap())
    {        
    }

    void setAcquisitionMode(const std::string& acquisitionMode)
    {        
        // Configure single frame acquisition on the camera:SingleFrame,Continuous
        CEnumParameter(nodemap_, "AcquisitionMode").SetValue(acquisitionMode.c_str() );
        std::cout << "[config param]: AcquisitionMode: " << CEnumParameter(nodemap_, "AcquisitionMode").GetValue() << std::endl;
    }

    void setAcquisitionFrameRate(double frameRate)
    {
        CBooleanParameter(nodemap_, "AcquisitionFrameRateEnable").SetValue(true);
        CFloatParameter(nodemap_, "AcquisitionFrameRateAbs").SetValue(frameRate);
        std::cout << "[config param]: AcquisitionFrameRateAbs: " <<  CFloatParameter(nodemap_, "AcquisitionFrameRateAbs").GetValue() << std::endl;
    }

    void setExposureTime(const float& time)
    {
        // Determine the current exposure time
        CFloatParameter ExposureTime_(nodemap_, "ExposureTime");        

        // Set the exposure time 
        ExposureTime_.SetValue(time);
        std::cout << "[config param]: latest exposure time: " << ExposureTime_.GetValue() << std::endl;
    }

    void setHardwareTrigger( const std::string& _hardware_trigger_source, const std::string& _hardware_trigger_edge)
    {
        //Disable compression mode.
        CConfigurationHelper::DisableCompression( nodemap_ );

        //Disable GenDC streaming.
        CConfigurationHelper::DisableGenDC( nodemap_ );

        //Select image component.
        CConfigurationHelper::SelectRangeComponent( nodemap_ );
        
        // Disable all trigger types except the trigger type used for triggering the acquisition of
        // frames.
        {
            // Get required enumerations.
            CEnumParameter triggerSelector( nodemap_, "TriggerSelector" );
            CEnumParameter triggerMode( nodemap_, "TriggerMode" );

            // Check the available camera trigger mode(s) to select the appropriate one: acquisition start trigger mode
            // (used by older cameras, i.e. for cameras supporting only the legacy image acquisition control mode;
            // do not confuse with acquisition start command) or frame start trigger mode
            // (used by newer cameras, i.e. for cameras using the standard image acquisition control mode;
            // equivalent to the acquisition start trigger mode in the legacy image acquisition control mode).
            String_t triggerName( "FrameStart" );
            if (!triggerSelector.CanSetValue( triggerName ))
            {
                triggerName = "AcquisitionStart";

                if (!triggerSelector.CanSetValue( triggerName ))
                {
                    throw RUNTIME_EXCEPTION( "Could not select trigger. Neither FrameStart nor AcquisitionStart is available." );
                }
            }
            
            // Get all enumeration entries of trigger selector.
            StringList_t triggerSelectorEntries;
            triggerSelector.GetSettableValues( triggerSelectorEntries );

            // Turn trigger mode off for all trigger selector entries except for the frame trigger given by triggerName.
            for (StringList_t::const_iterator it = triggerSelectorEntries.begin(); it != triggerSelectorEntries.end(); ++it)
            {
                // Set trigger mode to off.
                triggerSelector.SetValue( *it );
                
                if (triggerName == *it )
                {                   
                    // Activate trigger.
                    triggerMode.SetValue( "On" );                     
               
                    // The trigger source must be set to the trigger input, e.g. 'Line1'.
                    CEnumParameter(nodemap_, "TriggerSource").SetValue(_hardware_trigger_source.c_str());
					
                    ////The trigger activation must be set to e.g. 'RisingEdge'.
                    CEnumParameter(nodemap_, "TriggerActivation").SetValue(_hardware_trigger_edge.c_str());
                }
                else
                {
                    triggerMode.TrySetValue( "Off" );                    
                }
            }
            // Finally select the frame trigger type (resp. acquisition start type
            // for older cameras). Issuing a software trigger will now trigger
            // the acquisition of a frame.
            triggerSelector.SetValue( triggerName );
        }
    }

    void setImageROI(const int width, const int height, const int offsetX, const int offsetY)    
    {
        // Get the integer nodes describing the AOI.
        CIntegerParameter offsetX_( nodemap_, "OffsetX" );
        CIntegerParameter offsetY_( nodemap_, "OffsetY" );
        CIntegerParameter width_( nodemap_, "Width" );
        CIntegerParameter height_( nodemap_, "Height" );

        // On some cameras, the offsets are read-only.
        // Therefore, we must use "Try" functions that only perform the action
        // when parameters are writable. Otherwise, we would get an exception.
        offsetX_.TrySetValue(offsetX, IntegerValueCorrection_Nearest);
        offsetY_.TrySetValue(offsetY, IntegerValueCorrection_Nearest); 

        offsetX_.TrySetToMinimum();
        offsetY_.TrySetToMinimum(); 

        // Some properties have restrictions.
        // We use API functions that automatically perform value corrections.
        // Alternatively, you can use GetInc() / GetMin() / GetMax() to make sure you set a valid value.
        width_.SetValue( width, IntegerValueCorrection_Nearest );
        height_.SetValue( height, IntegerValueCorrection_Nearest ); 

        std::cout << "[config param]: OffsetX: " << offsetX_.GetValue() << std::endl;
        std::cout << "[config param]: OffsetY: " << offsetY_.GetValue() << std::endl;
        std::cout << "[config param]: Width: " << width_.GetValue() << std::endl;
        std::cout << "[config param]: Height: " << height_.GetValue() << std::endl; 
  
    }

    void setPixelFormat(const std::string& format)
    {

        CEnumParameter pixelFormat( nodemap_, "PixelFormat" );


        if (pixelFormat.CanSetValue( format.c_str() ))
        {
            pixelFormat.SetValue( format.c_str() );
            std::cout << "[config param]: latest PixelFormat: " << pixelFormat.GetValue() << std::endl;
        }
    }
		
    void setReverse(const bool reverseX = false, const bool reverseY = false)
	{

        auto reverseX_ = CBooleanParameter(nodemap_, "ReverseX");
        auto reverseY_ = CBooleanParameter(nodemap_, "ReverseY");

        if(reverseX)           
            reverseX_.TrySetValue(true);        
        else
            reverseX_.TrySetValue(false);

        if(reverseY)           
            reverseY_.TrySetValue(true);        
        else
            reverseY_.TrySetValue(false);
        
        std::cout<< "[config param]: set ReverseX: " << reverseX_.GetValue() << std::endl;
        std::cout<< "[config param]: set ReverseY: " << reverseY_.GetValue() << std::endl;			
		
	}

	void getDeviceTemperature(double& temperature)
	{

		temperature = CFloatParameter(nodemap_, "DeviceTemperature").GetValue();
		std::cout << "DeviceTemp: " << temperature << std::endl;
	}
	
	void getDeviceParamters(void)
	{

		std::cout << "Camera Device Information" << std::endl
			<< "=========================" << std::endl;
		std::cout << "Vendor           : "
			<< CStringParameter( nodemap_, "DeviceVendorName" ).GetValue() << std::endl;
		std::cout << "Model            : "
			<< CStringParameter( nodemap_, "DeviceModelName" ).GetValue() << std::endl;
		std::cout << "Firmware version : "
			<< CStringParameter( nodemap_, "DeviceFirmwareVersion" ).GetValue() << std::endl << std::endl;
		std::cout << "Serial Number : "
			<< CStringParameter( nodemap_, "DeviceSerialNumber" ).GetValue() << std::endl << std::endl;
		
		std::cout << "Camera Device Settings" << std::endl
			<< "======================" << std::endl;
		// Get the integer nodes describing the AOI.
		CIntegerParameter offsetX( nodemap_, "OffsetX" );
		CIntegerParameter offsetY( nodemap_, "OffsetY" );
		CIntegerParameter width( nodemap_, "Width" );
		CIntegerParameter height( nodemap_, "Height" );
		std::cout << "OffsetX          : " << offsetX.GetValue() << std::endl;
		std::cout << "OffsetY          : " << offsetY.GetValue() << std::endl;
		std::cout << "Width            : " << width.GetValue() << std::endl;
		std::cout << "Height           : " << height.GetValue() << std::endl;
	}

private:
    GENAPI_NAMESPACE::INodeMap& nodemap_;
};


class CHardwareTriggerConfiguration : public CConfigurationEventHandler
{
public:
    void ApplyConfiguration( GENAPI_NAMESPACE::INodeMap& nodemap )
    {
        using namespace GENAPI_NAMESPACE;

        CConfigurationHelper::DisableCompression( nodemap );


        CConfigurationHelper::DisableGenDC( nodemap );


        CConfigurationHelper::SelectRangeComponent( nodemap );

        {

            CEnumParameter triggerSelector( nodemap, "TriggerSelector" );
            CEnumParameter triggerMode( nodemap, "TriggerMode" );
            CFloatParameter triggerDelay( nodemap, "TriggerDelay");

            String_t triggerName( "FrameStart" );
            if (!triggerSelector.CanSetValue( triggerName ))
            {
                triggerName = "AcquisitionStart";
                if (!triggerSelector.CanSetValue( triggerName ))
                {
                    throw RUNTIME_EXCEPTION( "Could not select trigger. Neither FrameStart nor AcquisitionStart is available." );
                }
            }

            StringList_t triggerSelectorEntries;
            triggerSelector.GetSettableValues( triggerSelectorEntries );

            for (StringList_t::const_iterator it = triggerSelectorEntries.begin(); it != triggerSelectorEntries.end(); ++it)
            {
                
                triggerSelector.SetValue( *it );
                if (triggerName == *it)
                {
                    
                    triggerMode.SetValue( "On" );
                    std::cout<<"triggerMode: on"<<std::endl;

                    
                    CEnumParameter(nodemap, "TriggerSource").SetValue("Line4");
                    std::cout<<"TriggerSource: Line4"<<std::endl;
					
                    CEnumParameter(nodemap, "TriggerActivation").SetValue("AnyEdge");
                    std::cout<<"TriggerActivation: AnyEdge"<<std::endl;

                    break;
                }
                else
                {
                    triggerMode.TrySetValue( "Off" );
                    // std::cout<<"triggerMode: off"<<std::endl;
                }
            }
            
            triggerSelector.SetValue( triggerName );
            triggerDelay.SetValue(1.0);

        }

    }


    //Set basic camera settings.
    virtual void OnOpened( CInstantCamera& camera )
    {
        try
        {
            std::cout<<"ApplyConfiguration"<<std::endl;
            ApplyConfiguration( camera.GetNodeMap() );

        }
        catch (const GenericException& e)
        {
            throw RUNTIME_EXCEPTION( "Could not apply configuration. Pylon::GenericException caught in OnOpened method msg=%hs", e.what() );
        }
        catch (const std::exception& e)
        {
            throw RUNTIME_EXCEPTION( "Could not apply configuration. std::exception caught in OnOpened method msg=%hs", e.what() );
        }
        catch (...)
        {
            throw RUNTIME_EXCEPTION( "Could not apply configuration. Unknown exception caught in OnOpened method." );
        }
    }


};


//Example of an image event handler.
class CSampleImageEventHandler : public CImageEventHandler
{
public:
    CSampleImageEventHandler( camera_basler::CameraBasler* node ) : _node(node)
    {

    }

    virtual void OnImagesSkipped( CInstantCamera& camera, size_t countOfSkippedImages )
    {
        std::cout << "OnImagesSkipped event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        std::cout << countOfSkippedImages << " images have been skipped." << std::endl;
        std::cout << std::endl;
    }

    void OnImageGrabbed( CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult )
    {
        if (ptrGrabResult->GrabSucceeded())
        { 

            if(_node -> GetSaveImage())
            {
                
                storeImage(ptrGrabResult, _node -> _GetSavePath());
            } 
            else
            {

                auto ptr = std::make_unique<sensor_msgs::msg::Image>();

                auto h = ptr->height = ptrGrabResult->GetHeight();
                auto w = ptr->width = ptrGrabResult->GetWidth();
                
                ptr->encoding = "mono8";
                ptr->is_bigendian = false;
                ptr->step = w;
                ptr->data.resize(h * w);
                ptr->header.stamp = _node -> now();
                
                ptr->header.frame_id = _node -> GetFrameID();
                

                auto headI = (char*) (ptrGrabResult->GetBuffer());       
                auto head = (char*) (ptr->data.data());
                memcpy(head, headI, w * h );             
                
                _node -> PublishR(ptr);
                   
            }
            
                
        }
        else
        {
            _node -> GetFrameID();
            
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "[ %s ] : frameID: %s, getID: %d, Exception in camera image grabbed: %d, %s", _node->get_name(),_node->GetFrameIDStatic().c_str(),ptrGrabResult->GetID(), ptrGrabResult->GetErrorCode(), ptrGrabResult->GetErrorDescription().c_str());
        }
    }
private:
    
    camera_basler::CameraBasler* _node;

    void storeImage(const CGrabResultPtr& ptrGrabResult,const std::string& imgName)
	{
		auto height = ptrGrabResult->GetHeight();
		auto width = ptrGrabResult->GetWidth();		
		
		cv::Mat image(height,width,CV_8UC1);
		
		for(decltype(height) r = 0; r < height; ++r)
		{
			auto headI = (char*) (ptrGrabResult->GetBuffer()) + width * r;       
			auto head = (char*) (image.data) + width * r;
			memcpy(head, headI, width);        
		}
		bool ret = cv::imwrite(imgName,image);
	}
};



//------------------------------------ CameraBasler::_Impl ---------------------------------------------
namespace camera_basler
{

class CameraBasler::_Impl
{

public:
    explicit _Impl(CameraBasler* ptr) : _node(ptr)
    {
        _InitializeParameters();
        _UpdateParameters();
        _cameras.Initialize(_camera_number);
        _Config();

        impl_static_ptr = this;

    }

    ~_Impl()
    {
        // Releases all pylon resources.
        _Release();       
    }

    void Start() 
    {
        _UpdateParameters();
        _StartGrab();      
        
    }
 
    void Stop()  
    {
        _StopGrab();
        
    }

    void EnableSave(void)
    {
        _node -> SetSaveImage(true);        
		 RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"GetSaveImage()-0: %d", _node -> GetSaveImage());
    }

    void DisableSave(void)
    {
        _node -> SetSaveImage(false);
    }

    std::string GetCameraSerialNumber()
    {
        return _camera_serial_number;
    }

    std::string GetSavePath()
    {
        return _savePath;
    }

private:
    void _InitializeParameters()
    {
        _node->declare_parameter("serial_number_enable");
        _node->declare_parameter("camera_number");
        _node->declare_parameter("camera_serial_number");
        _node->declare_parameter("hardware_trigger_source");
        _node->declare_parameter("hardware_trigger_edge");

        _node->declare_parameter("acqusition_buffer_number");
        _node->declare_parameter("acqusition_mode");
        _node->declare_parameter("acqusition_frame_rate");
        _node->declare_parameter("exposure_time");

        _node->declare_parameter("offsetX");
        _node->declare_parameter("offsetY");
        _node->declare_parameter("width");
        _node->declare_parameter("height");
        _node->declare_parameter("pixelFormat");

        _node->declare_parameter("reverseX");
        _node->declare_parameter("reverseY");

        _node->declare_parameter("save_path");

    }

    void _UpdateParameters()
    {
        _node->get_parameter("serial_number_enable",_serial_number_enable);
        _node->get_parameter("camera_number",_camera_number);
        _node->get_parameter("camera_serial_number", _camera_serial_number);
        _node->get_parameter("hardware_trigger_source", _hardware_trigger_source);
        _node->get_parameter("hardware_trigger_edge", _hardware_trigger_edge);

        _node->get_parameter("acqusition_buffer_number", _acqusitionBufferNumber);
        _node->get_parameter("acqusition_mode", _acqusitionMode);
        _node->get_parameter("acqusition_frame_rate", _acqusitionFrameRate);
        _node->get_parameter("exposure_time", _exposureTime);

        _node->get_parameter("offsetX", _offsetX);
        _node->get_parameter("offsetY", _offsetY);
        _node->get_parameter("width", _width);
        _node->get_parameter("height", _height);
        _node->get_parameter("pixelFormat", _pixelFormat);     

        _node->get_parameter("reverseX",_reverseX);
        _node->get_parameter("reverseY",_reverseY);   

        _node->get_parameter("save_path",_savePath);
    }



    bool _RestartCamera()
    {
        _UpdateParameters();

        using namespace std::chrono_literals;

        for (size_t i = 0; i < _cameras.GetSize(); ++i)
        {
            if(_cameras[i].IsOpen ())
            {
                _cameras[i].Close();                
            }
        }      
        std::this_thread::sleep_for(50ms);
        for(size_t i = 0; i < _cameras.GetSize(); ++i)
        {               
            _cameras[i].Open();     
        }

        return true;

    }

    //TODO: camera config
    void _Config()
    {
        // Before using any pylon methods, the pylon runtime must be initialized.
        PylonInitialize();         

        if(_serial_number_enable)
        {
            _getSerialNumber();  
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"--->: to get camera's serial number, exit after the Node is executed"); 
        }
        
        //enumerate devices and start HardwareTrigger mode to grab
        {
            CTlFactory& TlFactory = CTlFactory::GetInstance();
            DeviceInfoList_t filter; 
            filter.push_back( CDeviceInfo().SetSerialNumber( _camera_serial_number.c_str() ) ); 

            DeviceInfoList_t lstDevices;
            TlFactory.EnumerateDevices( lstDevices, filter);
            std::cout << "lstDevices size: " << lstDevices.size() << std::endl;
            if ( lstDevices.empty() ) 
            {
                throw std::runtime_error("Error: No devices found!");
            }


            if(lstDevices.size() != _camera_number)
            {                
                throw std::runtime_error("Error: Cameras'number does not match!");
            }  

            for (size_t i = 0; i < _cameras.GetSize(); ++i)
            {               
                _cameras[i].Attach( TlFactory.CreateDevice( lstDevices[i] ) );               

                _cameras[i].RegisterConfiguration( new CHardwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete );
                
                _cameras[i].RegisterImageEventHandler(new CSampleImageEventHandler(this->_node), RegistrationMode_Append, Cleanup_Delete ); 
				

                _cameras[i].Open();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"camera: %s open",_cameras[i].GetDeviceInfo().GetSerialNumber().c_str());

                ConfigBasler configBasler(_cameras[i]);


                configBasler.setAcquisitionMode(_acqusitionMode);
                configBasler.setExposureTime(_exposureTime);                
                configBasler.setImageROI(_width, _height, _offsetX, _offsetY); 
                configBasler.setPixelFormat(_pixelFormat);               
                configBasler.setReverse(_reverseX,_reverseY);
                

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"camera: %s initialize successfully",_cameras[i].GetDeviceInfo().GetSerialNumber().c_str());
                
            }  
        }
    }

    void _StartGrab() 
    {
        for (size_t i = 0; i < _cameras.GetSize(); ++i)
        {
            if(_cameras[i].IsGrabbing())
            {
                return;
            }
            else
            {
                if(_cameras[i].CanWaitForFrameTriggerReady())
                    _cameras[i].StartGrabbing( GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera );
                else
                    throw std::runtime_error("camera serial number is not ready:" + _cameras[i].GetDeviceInfo().GetSerialNumber() );
            }
        }
    }

    void _StopGrab() 
    {		
        for (size_t i = 0; i < _cameras.GetSize(); ++i)
        {
            if(_cameras[i].IsGrabbing())
            {
                _cameras[i].StopGrabbing();
            }
        }
		
    
        std::cout<< "[" << _node -> get_name() << "]" << " frameID: "<< std::stoi(_node -> GetFrameID()) -1 << std::endl;

    /*  //输出一次测量相机触发次数数据到指定文件中，@NOTE: serial number需和测头basler相机对应    
        if(_camera_serial_number == "40138314")
        {
            //todo delete later
            std::ofstream of("/home/ubuntu/http_server/camera_l_frameID.txt", std::ofstream::out | std::ofstream::app);
            if(!of.is_open())
            {
                throw std::runtime_error("failed to open camera_l_frameID.txt");
            }
            else{
                RCLCPP_INFO(rclcpp::get_logger(_node->get_name()),"open file");
            }

            of << ++serial_number << ". " << _node -> get_name() << " frameID: " << _node->GetFrameIDStatic() << "\n";

            RCLCPP_INFO(rclcpp::get_logger(_node->get_name()),"write");

                       
            of.close();
            std::this_thread::sleep_for(5ms);

            RCLCPP_INFO(rclcpp::get_logger(_node->get_name()),"save info seccessfully");
        }
        else if(_camera_serial_number == "40136629")
        {
            
            std::ofstream of("/home/ubuntu/http_server/camera_r_frameID.txt", std::ofstream::out | std::ofstream::app);
            if(!of.is_open())
            {
                throw std::runtime_error("failed to open camera_r_frameID.txt");
            }
            else{
                RCLCPP_INFO(rclcpp::get_logger(_node->get_name()),"open file");
            }

            of << ++serial_number << ". " << _node -> get_name() << " frameID: " << _node->GetFrameIDStatic() << "\n";

            RCLCPP_INFO(rclcpp::get_logger(_node->get_name()),"write");


            of.close();
            std::this_thread::sleep_for(5ms);

            RCLCPP_INFO(rclcpp::get_logger(_node->get_name()),"save info seccessfully");
        }
        else
        {
            throw std::runtime_error("wrong serial number");
        }
    */
        _node -> ResetFrameID();

        

    }

    void _Release()
    {
        PylonTerminate();
    }

    void _getSerialNumber(void)
    {
        size_t c_maxCamerasToUse = 2;
        // Get the transport layer factory.
        CTlFactory& tlFactory = CTlFactory::GetInstance();

        // Get all attached devices and exit application if no device is found.
        DeviceInfoList_t devices;
        if (tlFactory.EnumerateDevices( devices ) == 0)
        {
            throw RUNTIME_EXCEPTION( "No camera present." );
        }

        // Create an array of instant cameras for the found devices and avoid exceeding a maximum number of devices.
        CInstantCameraArray cameras( std::min( devices.size(), c_maxCamerasToUse ) );

        // Create and attach all Pylon Devices.
        for (size_t i = 0; i < cameras.GetSize(); ++i)
        {
            cameras[i].Attach( tlFactory.CreateDevice( devices[i] ) );

            // Print the model name of the camera.
            std::cout << "Using device " << cameras[i].GetDeviceInfo().GetModelName() << std::endl;
            std::cout << "Serial Number : "                
                << cameras[i].GetDeviceInfo().GetSerialNumber() << std::endl;
        }       

        _Release();              
    }

private:
    int _acqusitionBufferNumber = 300;
    std::string _acqusitionMode = "Continuous";
    double _acqusitionFrameRate = 60.;
    double _exposureTime = 200.;

    int _offsetX = 0;               
    int _offsetY =  0;                       
    int _width = 1920;                         
    int _height = 1200;                          

    std::string _pixelFormat = "Mono8";          

    bool _serial_number_enable = false;
    int _camera_number = 0;
    std::string _camera_serial_number = "invalid serial number";    
    std::string _hardware_trigger_source = "line1";
    std::string _hardware_trigger_edge = "FallingEdge";

    int _reverseX = false;
    int _reverseY = false;

    std::string _savePath;

    CInstantCameraArray _cameras;

    CameraBasler* _node;

    bool saveImage;    

    static _Impl* impl_static_ptr;

    int serial_number = 0;
};

//------------------------------------ camera_basler ----------------------------------------------------
CameraBasler::CameraBasler(const rclcpp::NodeOptions& options) : Node("camera_basler_node", options)
{
    _init = std::thread(&CameraBasler::_Init, this);
}

CameraBasler::~CameraBasler()
{
    _init.join();

    _srvStart.reset();
    _srvStop.reset();
    _impl.reset();
    _pubR.reset();
    _srvEnableSave.reset();
    _srvDisableSave.reset();


    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "camera_basler destroyed successfully");
}

CameraBasler::_Impl* CameraBasler::_Impl::impl_static_ptr = nullptr;

void CameraBasler::_Init() try
{
    _pubR = this->create_publisher<sensor_msgs::msg::Image>(_pubRName, 50);

    _impl = std::make_unique<_Impl>(this);
    

    _srvStart = this->create_service<std_srvs::srv::Trigger>(_srvStartName, std::bind(&CameraBasler::_Start, this, std::placeholders::_1, std::placeholders::_2));
    _srvStop = this->create_service<std_srvs::srv::Trigger>(_srvStopName, std::bind(&CameraBasler::_Stop, this, std::placeholders::_1, std::placeholders::_2));
    _srvEnableSave = this->create_service<std_srvs::srv::Trigger>(_srvEnableSaveName, std::bind(&CameraBasler::_EnableSave, this, std::placeholders::_1, std::placeholders::_2));
    _srvDisableSave = this->create_service<std_srvs::srv::Trigger>(_srvDisableSaveName,std::bind(&CameraBasler:: _DisableSave,this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "camera_basler initialized successfully");
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_basler initializer:%s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_basler initializer:%s", "unknown");
    rclcpp::shutdown();
}

void CameraBasler::_Start(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) try //useless
{
    response->success = false;
    response->message = "Fail: camera start";

    _impl->Start();

    response->success = true;
    response->message = "Success: camera start";
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_basler service start:%s", e.what());
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_basler service start:%s", "unknown");
}
 
void CameraBasler::_Stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) try //useless
{
    response->success = false;
    response->message = "Fail: camera stop";

    _impl->Stop();

    auto ptrR = std::make_unique<sensor_msgs::msg::Image>();

    ptrR->header.stamp = now();

    ptrR->header.frame_id = "-1";

    PublishR(ptrR);
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()),"publish img, header.frame_id = -1");

    response->success = true;
    response->message = "Success: camera stop";
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_basler stop:%s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_basler stop:%s", "unknown");
    rclcpp::shutdown();
}

void CameraBasler::_EnableSave(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) try
{
    response->success = false;
    response->message = "Fail: enable save image";

    _impl->EnableSave();

    response->success = true;
    response->message = "Success: enable save image";

    
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_basler stop:%s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_basler stop:%s", "unknown");
    rclcpp::shutdown();
}

void CameraBasler::_DisableSave(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) try
{
    response->success = false;
    response->message = "Fail: disable save image";

    _impl->DisableSave();

    response->success = true;
    response->message = "Success: disable save image";


}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_basler stop:%s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_basler stop:%s", "unknown");
    rclcpp::shutdown();
}

std::string CameraBasler::_GetCameraSerialNumber()
{
    return _impl->GetCameraSerialNumber();
}

std::string CameraBasler::_GetSavePath()
{
    return _impl->GetSavePath();
}



}//namespace camera_basler



#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_basler::CameraBasler)
