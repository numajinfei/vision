/*
 * @Descripttion: 
 * @version: 
 * @Author: hw
 * @Date: 2021-09-13 16:32:14
 * @LastEditors: hw
 * @LastEditTime: 2021-10-18 23:51:27
 */
#include "camera_basler/camera_basler.hpp"

#include <exception>

#include "opencv2/core.hpp" //opencv to store images
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

using namespace std::chrono_literals;

#include <pylon/PylonIncludes.h>
// #include <pylon/SoftwareTriggerConfiguration.h> //CSoftwareTriggerConfiguration
// #include "camera_basler/ConfigurationEventPrinter.h"

camera_basler::CameraBasler * nodeptr = nullptr;

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using GenApi objects.
using namespace GenApi;

namespace Pylon
{
class ConfigBasler
{
public:
    ConfigBasler(CInstantCamera& camera) : nodemap_(camera.GetNodeMap())
    {        
    }

    void setAcquisitionFrameRate(double frameRate)
    {
        CBooleanParameter(nodemap_, "AcquisitionFrameRateEnable").SetValue(true);
        CFloatParameter(nodemap_, "AcquisitionFrameRateAbs").SetValue(frameRate);
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
        offsetX_.TrySetToMinimum();
        offsetY_.TrySetToMinimum();

        // Some properties have restrictions.
        // We use API functions that automatically perform value corrections.
        // Alternatively, you can use GetInc() / GetMin() / GetMax() to make sure you set a valid value.
        width_.SetValue( width, IntegerValueCorrection_Nearest );
        height_.SetValue( height, IntegerValueCorrection_Nearest );

        std::cout << "OffsetX          : " << offsetX_.GetValue() << std::endl;
        std::cout << "OffsetY          : " << offsetY_.GetValue() << std::endl;
        std::cout << "Width            : " << width_.GetValue() << std::endl;
        std::cout << "Height           : " << height_.GetValue() << std::endl;   
    }

    void setPixelFormat(const std::string& format)
    {
        // Access the PixelFormat enumeration type node.
        CEnumParameter pixelFormat( nodemap_, "PixelFormat" );

        // Remember the current pixel format.
        String_t oldPixelFormat = pixelFormat.GetValue();
        std::cout << "Old PixelFormat  : " << oldPixelFormat << std::endl;

        // Set the pixel format to Mono8 if available.
        if (pixelFormat.CanSetValue( format.c_str() ))
        {
            pixelFormat.SetValue( format.c_str() );
            std::cout << "New PixelFormat  : " << pixelFormat.GetValue() << std::endl;
        }
    }
		
	void getDeviceTemperature(double& temperature)//TODO:verify
	{
		// Get the current device temperature
		temperature = CFloatParameter(nodemap_, "DeviceTemperature").GetValue();
		std::cout << "DeviceTemp: " << temperature << std::endl;
	}
	
	void getDeviceParamters(void)
	{
		//INodeMap& nodemap = camera.GetNodeMap();
		// Get camera device information.
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
		//std::cout << "DeviceSerialNumber : " 
		//	<< camera.GetDeviceInfo().GetSerialNumber() << std::endl; //sure the device is open.
			
		// Camera settings.
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
	
	
	
	

/*
    void setGain(float percent)
    {
        // Set the new gain to 50% ->  Min + ((Max-Min) / 2).
        //
        // Note: Some newer camera models may have auto functions enabled.
        //       To be able to set the gain value to a specific value
        //       the Gain Auto function must be disabled first.
        // Access the enumeration type node GainAuto.
        // We use a "Try" function that only performs the action if the parameter is writable.
        CEnumParameter gainAuto( nodemap_, "GainAuto" );
        gainAuto.TrySetValue( "Off" );

        // Check to see which Standard Feature Naming Convention (SFNC) is used by the camera device.
        if (camera.GetSfncVersion() >= Sfnc_2_0_0)
        {
            // Access the Gain float type node. This node is available for USB camera devices.
            // USB camera devices are compliant to SFNC version 2.0.
            CFloatParameter gain( nodemap_, "Gain" );
            gain.SetValuePercentOfRange( percent );
            std::cout << "Gain (50%)       : " << gain.GetValue() << " (Min: " << gain.GetMin() << "; Max: " << gain.GetMax() << ")" << std::endl;
        }
        else
        {
            // Access the GainRaw integer type node. This node is available for IIDC 1394 and GigE camera devices.
            CIntegerParameter gainRaw( nodemap_, "GainRaw" );
            gainRaw.SetValuePercentOfRange( percent );
            std::cout << "Gain (50%)       : " << gainRaw.GetValue() << " (Min: " << gainRaw.GetMin() << "; Max: " << gainRaw.GetMax() << "; Inc: " << gainRaw.GetInc() << ")" << std::endl;
        }

    }
*/


    void setExposureTime(const float& time)//TODO:verify
    {
        // Determine the current exposure time
        double d = CFloatParameter(nodemap_, "ExposureTime").GetValue();
        std::cout << " current exposure time : " << d << std::endl;

        // Set the exposure time to 3500 microseconds
        CFloatParameter(nodemap_, "ExposureTime").SetValue(time);
        std::cout << "Setting exposure time: " << CFloatParameter(nodemap_, "ExposureTime").GetValue() << std::endl;
    }

    void setAcquisitionMode(const std::string& acquisitionMode)
    {
        // INodeMap& nodemap = camera.GetNodeMap();
        // Configure single frame acquisition on the camera:SingleFrame,Continuous
        CEnumParameter(nodemap_, "AcquisitionMode").SetValue(acquisitionMode.c_str() );
    }

	void setReverse(bool X_enable, bool Y_enable)
	{
		if(X_enable == false)
			CBooleanParameter(nodemap_, "ReverseX").SetValue(false);
		else 
        {
            CBooleanParameter(nodemap_, "ReverseX").SetValue(true);
		    // std::cout << "ReverseX: " << CBooleanParameter(nodemap_, "ReverseX").GetValue() << std::endl;
            if(!CBooleanParameter(nodemap_, "ReverseX").GetValue())
                throw std::runtime_error("ReverseX failed");
        }
			
		
		if(Y_enable == false)
			CBooleanParameter(nodemap_, "ReverseY").SetValue(false);
		else
        {
            CBooleanParameter(nodemap_, "ReverseY").SetValue(true);
            // std::cout << "ReverseY: " << CBooleanParameter(nodemap_, "ReverseY").GetValue() << std::endl;
            if(!CBooleanParameter(nodemap_, "ReverseY").GetValue())
                throw std::runtime_error("ReverseY failed");
        }
			
		
	}

    void setHardwareTrigger(const std::string& linex, const std::string& triggerEdge)
    {
        // INodeMap& nodemap = camera.GetNodeMap();

        //1. Line mode:
        // Get the current line mode
        String_t e = CEnumParameter(nodemap_, "LineMode").GetValue();
        std::cout << "LineMode0 : " << e << std::endl;
        // Select GPIO line 3
std::cout << "hwTrriger - 1...\n";		
        CEnumParameter(nodemap_, "LineSelector").SetValue(linex.c_str());
        // Set the line mode to Input,Output or InOut
std::cout << "hwTrriger - 2...\n";			
        CEnumParameter(nodemap_, "LineMode").SetValue("Input");
        // Get the current line mode
std::cout << "hwTrriger - 3...\n";			
        e = CEnumParameter(nodemap_, "LineMode").GetValue();
        std::cout << "LineMode1 : " << e << std::endl;
std::cout << "hwTrriger - 4...\n";	
        //2.Line Debouncer 纭欢瑙﹀彂淇″彿娑堟姈璁剧疆 daA1920-160um涓嶆敮鎸?
        // INodeMap& nodemap = camera.GetNodeMap();
        // // Select the desired input line
        // CEnumParameter(nodemap, "LineSelector").SetValue("Line1");
        // // Set the parameter value to 10 microseconds
        // CFloatParameter(nodemap, "LineDebouncerTime").SetValue(10.0);

        //3. trigger source
        // Select the Frame Start trigger		
        CEnumParameter triggerSelector( nodemap_, "TriggerSelector" );
std::cout << "hwTrriger - 5...\n";		
        // CEnumParameter(nodemap, "TriggerSelector").SetValue("FrameStart");
        if (!triggerSelector.CanSetValue( "FrameStart" ))
        {
            if (!triggerSelector.CanSetValue( "AcquisitionStart" ))
            {
                throw RUNTIME_EXCEPTION( "Could not select trigger. Neither FrameStart nor AcquisitionStart is available." );
            }
        }
		//CEnumParameter(nodemap_, "TriggerSelector").SetValue("FrameStart");
std::cout << "hwTrriger - 6...\n";		
        // Set the trigger source to LineX
        CEnumParameter(nodemap_, "TriggerSource").SetValue(linex.c_str());
std::cout << "hwTrriger - 7...\n";
        //4. Trigger Action:RisingEdge,FallingEdge,AnyEdge,LevelHigh,LevelLow
        CEnumParameter(nodemap_, "TriggerActivation").SetValue(triggerEdge.c_str());
std::cout << "hwTrriger - 8...\n";
        //5. Trigger Delay
        // Set the delay for the frame start trigger to 100 碌s
        //CIntegerParameter(nodemap_, "TriggerDelay").SetValue(100);
std::cout << "hwTrriger - 9...\n";		
    }

    void setSoftwareTrigger()
    {
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
                if (triggerName == *it)
                {
                    // Activate trigger.
                    triggerMode.SetValue( "On" );

                    // The trigger source must be set to 'Software'.
                    CEnumParameter( nodemap_, "TriggerSource" ).SetValue( "Software" );

                    //// Alternative hardware trigger configuration:
                    //// This configuration can be copied and modified to create a hardware trigger configuration.
                    //// Remove setting the 'TriggerSource' to 'Software' (see above) and
                    //// use the commented lines as a starting point.
                    //// The camera user's manual contains more information about available configurations.
                    //// The Basler pylon Viewer tool can be used to test the selected settings first.

                    // // Select the desired input line
                    // CEnumParameter(nodemap, "LineSelector").SetValue("Line1");
                    // // Set the parameter value to 10 microseconds
                    // CFloatParameter(nodemap, "LineDebouncerTime").SetValue(10.0);
                    // // The trigger source must be set to the trigger input, e.g. 'Line1'.
                    // CEnumParameter(nodemap, "TriggerSource").SetValue("Line1");

                    // //The trigger activation must be set to e.g. 'RisingEdge'.
                    // CEnumParameter(nodemap, "TriggerActivation").SetValue("RisingEdge");
                    // // Set the delay for the frame start trigger to 300 碌s
                    // CFloatParameter(nodemap, "TriggerDelayAbs").SetValue(100.0);
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

private:
    GENAPI_NAMESPACE::INodeMap& nodemap_;
};



class CHardwareTriggerConfiguration : public CConfigurationEventHandler
{
public:
    /// Apply software trigger configuration.
    static void ApplyConfiguration( GENAPI_NAMESPACE::INodeMap& nodemap )
    {
        using namespace GENAPI_NAMESPACE;

        //Disable compression mode.
        CConfigurationHelper::DisableCompression( nodemap );

        //Disable GenDC streaming.
        CConfigurationHelper::DisableGenDC( nodemap );

        //Select image component.
        CConfigurationHelper::SelectRangeComponent( nodemap );

        // Disable all trigger types except the trigger type used for triggering the acquisition of
        // frames.
        {
            // Get required enumerations.
            CEnumParameter triggerSelector( nodemap, "TriggerSelector" );
            CEnumParameter triggerMode( nodemap, "TriggerMode" );

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
                if (triggerName == *it)
                {
                    // Activate trigger.
                    triggerMode.SetValue( "On" );

                    // The trigger source must be set to 'Software'.
                    //CEnumParameter( nodemap, "TriggerSource" ).SetValue( "Software" );

                    //// Alternative hardware trigger configuration:
                    //// This configuration can be copied and modified to create a hardware trigger configuration.
                    //// Remove setting the 'TriggerSource' to 'Software' (see above) and
                    //// use the commented lines as a starting point.
                    //// The camera user's manual contains more information about available configurations.
                    //// The Basler pylon Viewer tool can be used to test the selected settings first.

                    //// The trigger source must be set to the trigger input, e.g. 'Line1'.
                    CEnumParameter(nodemap, "TriggerSource").SetValue("Line4");
					//CEnumParameter(nodemap, "LineMode").SetValue("Input");

                    ////The trigger activation must be set to e.g. 'RisingEdge'.
                    CEnumParameter(nodemap, "TriggerActivation").SetValue("AnyEdge");
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


        //Set acquisition mode to "continuous"
        CEnumParameter( nodemap, "AcquisitionMode" ).SetValue( "Continuous" );
    }

    static void ApplyConfiguration1( CInstantCamera& camera )
    {
        //Disable compression mode.
        CConfigurationHelper::DisableCompression( camera.GetNodeMap() );
        //Disable GenDC streaming.
        CConfigurationHelper::DisableGenDC( camera.GetNodeMap() );
        //Select image component.
        CConfigurationHelper::SelectRangeComponent( camera.GetNodeMap() );
std::cout << "[apply_cfg]-1 ..." << std::endl;
        ConfigBasler configBasler(camera);
std::cout << "[apply_cfg]-2 ..." << std::endl;		
        //configBasler.setAcquisitionFrameRate(90.0);
std::cout << "[apply_cfg]-3 ..." << std::endl;		
        configBasler.setImageROI(1920, 1200, 0, 0);
		//configBasler.getDeviceParamters();
		double temp = 0.0;
		configBasler.getDeviceTemperature(temp);
std::cout << "[apply_cfg]-4 ..." << std::endl;		
        configBasler.setPixelFormat("Mono8");
std::cout << "[apply_cfg]-5 ..." << std::endl;		
        configBasler.setHardwareTrigger("Line4", "FallingEdge");
std::cout << "[apply_cfg]-6 ..." << std::endl;		
        //configBasler.setAcquisitionMode("SingleFrame");
std::cout << "[apply_cfg]-7 ..." << std::endl;	
        
    }

    //Set basic camera settings.
    virtual void OnOpened( CInstantCamera& camera )
    {
        try
        {
            ApplyConfiguration( camera.GetNodeMap() );
            //ApplyConfiguration1(camera);
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
    virtual void OnImagesSkipped( CInstantCamera& camera, size_t countOfSkippedImages )
    {
        std::cout << "OnImagesSkipped event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        std::cout << countOfSkippedImages << " images have been skipped." << std::endl;
        std::cout << std::endl;
    }

    void OnImageGrabbed( CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult )
    {
        std::cout << "CSampleImageEventHandler::OnImageGrabbed called." << std::endl;

        //Image grabbed successfully?
        if (ptrGrabResult->GrabSucceeded())
        {
            

            auto ptr = std::make_unique<sensor_msgs::msg::Image>();

            auto h = ptr->height = ptrGrabResult->GetHeight();
		    auto w = ptr->width = ptrGrabResult->GetWidth();

            
            ptr->encoding = "mono8";
            ptr->is_bigendian = false;
            ptr->step = w;
            ptr->data.resize(h * w);
            ptr->header.stamp = nodeptr->now();
            ptr->header.frame_id = ptrGrabResult->GetID();

            for(decltype(h) r = 0; r < h; ++r)
            {
                auto headI = (char*) (ptrGrabResult->GetBuffer()) + w * r;       
                auto head = (char*) (ptr->data.data()) + w * r;
                memcpy(head, headI, w);        
            }

            if(camera.GetDeviceInfo().GetSerialNumber() == nodeptr->_GetCamera1SerialNumber().c_str())
                nodeptr->PublishL(ptr);
            else
                nodeptr->PublishR(ptr);

        }
        else
        {
            std::cout << "Error: " << std::hex << ptrGrabResult->GetErrorCode() << std::dec << " " << ptrGrabResult->GetErrorDescription() << std::endl;
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in camera image grabbed: %d, %s", ptrGrabResult->GetErrorCode(), ptrGrabResult->GetErrorDescription());
        }
    }
private:
	void storeImage(const CGrabResultPtr& ptrGrabResult)
	{
		static bool camera_l = true;
		if(camera_l == false)
			return;
		auto h = ptrGrabResult->GetHeight();
		auto w = ptrGrabResult->GetWidth();
		
		//std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
		cv::Mat image(h,w,CV_8UC1);
		
		for(decltype(h) r = 0; r < h; ++r)
		{
			auto headI = (char*) (ptrGrabResult->GetBuffer()) + w * r;       
			auto head = (char*) (image.data) + w * r;
			memcpy(head, headI, w);        
		}
		bool ret = cv::imwrite("./image.bmp",image);
		camera_l = false;
		/* 
		if(ret)
		{
			std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
			std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end-start);
			std::cout << "it took " << time_span.count() << " seconds" << std::std::endl;
			cv::namedWindow("image");
			cv::resizeWindow("image",800,600);
			cv::imshow("image",image);
			cv::waitKey(80);
		}
		*/
	}
};

}

namespace camera_basler
{
//------------------------------------ CameraBasler::_Impl ---------------------------------------------

class CameraBasler::_Impl
{
public:
    explicit _Impl(CameraBasler* ptr) : _node(ptr)
    {
        _InitializeParameters();
        _Config();

        nodeptr = _node;
    }

    ~_Impl()
    {
        // Releases all pylon resources.
        PylonTerminate();
    }

    void Start()
    {
        _UpdateParameters();
        // _TriggerOn();
        _Config();
        
    }

    void Stop()
    {
        // for(size_t i = 0; i < 2; ++i)
        // {
        //     _cameras[i].AcquisitionStop.Execute();
        // }
    }

    std::string GetCamera1SerialNumber()
    {
        return _camera_1_serial_number;
    }
private:
    void _InitializeParameters()
    {
        _node->declare_parameter("acqusition_buffer_number");
        _node->declare_parameter("acqusition_frame_rate");
        _node->declare_parameter("exposure_time");

        _node->declare_parameter("serial_number_enbale");
        _node->declare_parameter("camera_1_serial_number");
        _node->declare_parameter("camera_2_serial_number");
        _node->declare_parameter("hardware_trigger_source");
        _node->declare_parameter("hardware_trigger_edge");
    }

    void _UpdateParameters()
    {
        _node->get_parameter("acqusition_buffer_number", _acqusitionBufferNumber);
        _node->get_parameter("acqusition_frame_rate", _acqusitionFrameRate);
        _node->get_parameter("exposure_time", _exposureTime);

        _node->get_parameter("camera_1_serial_number", _camera_1_serial_number);
        _node->get_parameter("camera_2_serial_number", _camera_2_serial_number);
        _node->get_parameter("hardware_trigger_source", _hardware_trigger_source);
        _node->get_parameter("hardware_trigger_edge", _hardware_trigger_edge);
    }

    //TODO: camera config
    void _Config()
    {
        // Before using any pylon methods, the pylon runtime must be initialized.
        PylonInitialize();

        if(_serial_number_enbale != false)
        {
            _getSerialNumber();
        }
        
        //enumerate devices and start HardwareTrigger mode to grab
        //{        
            CTlFactory& TlFactory = CTlFactory::GetInstance();
            DeviceInfoList_t filter; 
            filter.push_back( CDeviceInfo().SetSerialNumber( _camera_1_serial_number.c_str() ) ); //"0815-0000"
            filter.push_back( CDeviceInfo().SetSerialNumber( _camera_2_serial_number.c_str() ) ); //"0815-0001"

            DeviceInfoList_t lstDevices;
            TlFactory.EnumerateDevices( lstDevices, filter);
            std::cout << "lstDevices.size: " << lstDevices.size() << std::endl;
            if ( lstDevices.empty() ) {
                throw std::runtime_error("No devices found!");
            }

            DeviceInfoList_t::const_iterator it;
            for ( it = lstDevices.begin(); it != lstDevices.end(); ++it )
            {
                std::cout << it->GetFullName() << std::endl;
            }

            if(lstDevices.size() != 2 )
            {
                throw std::runtime_error("cameras'number does not match!");
            }
            CInstantCameraArray _cameras( lstDevices.size());

            for (size_t i = 0; i < _cameras.GetSize(); ++i)
            {
                _cameras[i].Attach( TlFactory.CreateDevice( lstDevices[i] ) );
                
                

                //_cameras[i].RegisterConfiguration( new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete );
                _cameras[i].RegisterConfiguration( new CHardwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete );
                _cameras[i].RegisterConfiguration( new CConfigurationEventPrinter, RegistrationMode_Append, Cleanup_Delete );
                _cameras[i].RegisterImageEventHandler( new CSampleImageEventHandler, RegistrationMode_Append, Cleanup_Delete );

                _cameras[i].Open();
                
                //Due to the structural design, the left camera had to be reversed.
                if(_cameras[i].GetDeviceInfo().GetSerialNumber() == nodeptr->_GetCamera1SerialNumber().c_str())
                {
                    ConfigBasler ConfigBasler(_cameras[i]);                    
                    ConfigBasler.setReverse(true, true);                    
                    std::cout << "camera 1 serialNumber: " << _cameras[i].GetDeviceInfo().GetSerialNumber() << std::endl;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "camera 1 reserveX,Y successfully");
                }
                
                if(_cameras[i].CanWaitForFrameTriggerReady())
                {
                    _cameras[i].StartGrabbing( GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera );
                }
                else
                {
                    throw std::runtime_error("camera serial number is not ready:" + _cameras[i].GetDeviceInfo().GetSerialNumber() );
                }
            }                                                       
        //}
        
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
                //<< CStringParameter( nodemap_, "DeviceSerialNumber" ).GetValue() << std::endl << std::endl;
                << cameras[i].GetDeviceInfo().GetSerialNumber() << std::endl;
        }                     
    }

private:
    int _acqusitionBufferNumber = 300;
    double _acqusitionFrameRate = 60.;
    double _exposureTime = 200.;

    bool _serial_number_enbale = false;
    std::string _camera_1_serial_number = "invalid serial number";
    std::string _camera_2_serial_number = "invalid serial number";
    std::string _hardware_trigger_source = "line1";
    std::string _hardware_trigger_edge = "FallingEdge";

    // CInstantCameraArray _cameras(2);

    // CInstantCamera _cameras(2);

    CameraBasler* _node;
};

//------------------------------------ camera_basler ----------------------------------------------------
CameraBasler::CameraBasler(const rclcpp::NodeOptions& options) : Node("cameta_basler_node", options)
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
    _pubL.reset();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "camera_basler destroyed successfully");
}

void CameraBasler::_Init() try
{
    _pubL = this->create_publisher<sensor_msgs::msg::Image>(_pubLName, 50);
    _pubR = this->create_publisher<sensor_msgs::msg::Image>(_pubRName, 50);

    _impl = std::make_unique<_Impl>(this);

    _srvStart = this->create_service<std_srvs::srv::Trigger>(_srvStartName, std::bind(&CameraBasler::_Start, this, std::placeholders::_1, std::placeholders::_2));
    _srvStop = this->create_service<std_srvs::srv::Trigger>(_srvStopName, std::bind(&CameraBasler::_Stop, this, std::placeholders::_1, std::placeholders::_2));

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

void CameraBasler::_Start(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) try
{
    response->success = false;
    response->message = "Fail: camer start";

    _impl->Start();

    response->success = true;
    response->message = "Success: cameta start";
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_basler service start:%s", e.what());
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_basler service start:%s", "unknown");
}

void CameraBasler::_Stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) try
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
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_basler stop:%s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in camera_basler stop:%s", "unknown");
    rclcpp::shutdown();
}

std::string CameraBasler::_GetCamera1SerialNumber()
{
    return _impl->GetCamera1SerialNumber();
}


}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_basler::CameraBasler)