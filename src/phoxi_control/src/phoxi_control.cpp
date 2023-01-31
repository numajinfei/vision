#include "phoxi_control/phoxi_control.hpp"


// #include "opencv2/core.hpp"
#include "opencv2/core/traits.hpp" //must be put after "photoneo.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp" 

#include <assert.h>
#include <exception> 
#include <chrono>  
#include <typeinfo>
#include <string>
#include <fstream>
#include <ctime>

using namespace std::chrono_literals;

namespace phoxi_control
{

const char* GetLocalTime()
{
    time_t now = time(0);
    std::tm* localTime = std::localtime(&now);
    static const std::string time = "[" + std::to_string(1900 + localTime->tm_year) + "-" 
            + std::to_string(1 + localTime->tm_mon) + "-"
            + std::to_string(localTime->tm_mday) + " "
            + std::to_string(localTime->tm_hour) + ":"
            + std::to_string(localTime->tm_min) + ":"
            + std::to_string(localTime->tm_sec) + "]";
    static const char* time_ptr = time.c_str();
    return time_ptr;
}

class PhoXiControl::_Impl
{
    public:
        explicit _Impl(PhoXiControl* ptr) : _node(ptr) 
        {
            try
            {
                _InitializeParameters();
                _UpdateParameters();

                GetAvailableDevices();
                ConnectPhoXiDevice();
                ChangeSettings();
                StartAcquisition();
            }
            catch(const std::exception& e)
            {
                RCLCPP_INFO(_node->get_logger(), "%s Error in _Impl(): %s", GetLocalTime(), e.what());
                rclcpp::shutdown();
            }
            catch(...)
            {
                RCLCPP_INFO(_node->get_logger(), "%s Error in _Impl(): unknown.", GetLocalTime());
                rclcpp::shutdown();
            }

        }


        ~_Impl()
        {

        }
     
        void _InitializeParameters()
        {
            _node->declare_parameter("width");
            _node->declare_parameter("height");
            _node->declare_parameter("triggerMode");
            _node->declare_parameter("timeOut");
            _node->declare_parameter("shutterMultiplier");
            _node->declare_parameter("scanMultiplier");
            _node->declare_parameter("cameraOnlyMode"); 
            _node->declare_parameter("ambientLightSuppression");
            _node->declare_parameter("maximumFPS");
            _node->declare_parameter("singlePatternExposure");

            _node->declare_parameter("codingStrategy");
            _node->declare_parameter("codingQuality");
            _node->declare_parameter("textureSource");
            _node->declare_parameter("laserPower");
            _node->declare_parameter("lEDPower");
            _node->declare_parameter("projectionOffsetLeft");
            _node->declare_parameter("projectionOffsetRight");
            _node->declare_parameter("confidence");
            _node->declare_parameter("roiSpace");
            _node->declare_parameter("minX"); 

            _node->declare_parameter("minY");
            _node->declare_parameter("minZ");
            _node->declare_parameter("maxX");
            _node->declare_parameter("maxY");
            _node->declare_parameter("maxZ");
            _node->declare_parameter("maxCameraAngle");
            _node->declare_parameter("maxProjectorAngle");
            _node->declare_parameter("minHalfwayAngle");
            _node->declare_parameter("maxHalfwayAngle");
            _node->declare_parameter("surfaceSmothness");

            _node->declare_parameter("calibrationVolumeOnly");
            _node->declare_parameter("normalsEstimationRadius");
            _node->declare_parameter("interreflectionsFiltering");
            _node->declare_parameter("interreflectionFilterStrength");

            _node->declare_parameter("invertedMarkers");
            _node->declare_parameter("markerScaleW");
            _node->declare_parameter("markerScaleH");
            _node->declare_parameter("coordinateSpace");   
            _node->declare_parameter("rotationCustom");
            _node->declare_parameter("translationCustom");  

            _node->declare_parameter("rotationRobot");
            _node->declare_parameter("translationRobot");  
            _node->declare_parameter("recognizeMarkers"); 
            _node->declare_parameter("connectionMethod");
            _node->declare_parameter("HWIdentification");   
            _node->declare_parameter("fileCameraFolder");
            _node->declare_parameter("outputFolder"); 
            _node->declare_parameter("projectName");
            _node->declare_parameter("saveFrame");

        }

        void _UpdateParameters()
        {
            _node->get_parameter("width", width);
            _node->get_parameter("height", height);
            _node->get_parameter("triggerMode", triggerMode);
            _node->get_parameter("timeOut", timeOut);
            _node->get_parameter("shutterMultiplier", shutterMultiplier);
            _node->get_parameter("scanMultiplier", scanMultiplier);
            _node->get_parameter("cameraOnlyMode", cameraOnlyMode);
            _node->get_parameter("ambientLightSuppression", ambientLightSuppression);
            _node->get_parameter("maximumFPS", maximumFPS);
            _node->get_parameter("singlePatternExposure", singlePatternExposure);

            _node->get_parameter("codingStrategy", codingStrategy);
            _node->get_parameter("codingQuality", codingQuality);
            _node->get_parameter("textureSource", textureSource);
            _node->get_parameter("laserPower", laserPower);
            _node->get_parameter("lEDPower", lEDPower);
            _node->get_parameter("projectionOffsetLeft", projectionOffsetLeft);
            _node->get_parameter("projectionOffsetRight", projectionOffsetRight);
            _node->get_parameter("confidence", confidence);
            _node->get_parameter("roiSpace", roiSpace);
            _node->get_parameter("minX", minX);

            _node->get_parameter("minY", minY);
            _node->get_parameter("minZ", minZ);
            _node->get_parameter("maxX", maxX);
            _node->get_parameter("maxY", maxY);
            _node->get_parameter("maxZ", maxZ);
            _node->get_parameter("maxCameraAngle", maxCameraAngle);
            _node->get_parameter("maxProjectorAngle", maxProjectorAngle);
            _node->get_parameter("minHalfwayAngle", minHalfwayAngle);
            _node->get_parameter("maxHalfwayAngle", maxHalfwayAngle);
            _node->get_parameter("surfaceSmothness", surfaceSmothness);

            _node->get_parameter("calibrationVolumeOnly", calibrationVolumeOnly);
            _node->get_parameter("normalsEstimationRadius", normalsEstimationRadius);
            _node->get_parameter("interreflectionsFiltering", interreflectionsFiltering);
            _node->get_parameter("interreflectionFilterStrength", interreflectionFilterStrength);
            _node->get_parameter("invertedMarkers", invertedMarkers);
            _node->get_parameter("markerScaleW", markerScaleW);
            _node->get_parameter("markerScaleH", markerScaleH);
            _node->get_parameter("coordinateSpace", coordinateSpace);
            _node->get_parameter("rotationCustom", rotationCustom);
            _node->get_parameter("translationCustom", translationCustom);

            _node->get_parameter("rotationRobot", rotationRobot);
            _node->get_parameter("translationRobot", translationRobot);
            _node->get_parameter("recognizeMarkers", recognizeMarkers);
            _node->get_parameter("connectionMethod", connectionMethod); 
            _node->get_parameter("HWIdentification", HWIdentification);
            _node->get_parameter("fileCameraFolder", fileCameraFolder);
            _node->get_parameter("outputFolder", outputFolder);
            _node->get_parameter("projectName", projectName);
            _node->get_parameter("saveFrame", saveFrame);

        }   

        void GetAvailableDevices()
        {
            //Wait for the PhoXiControl
            unsigned long times_connectPhoXiControl = 0;
            while (!Factory.isPhoXiControlRunning()) //tips: Running PhoXiControl is necessary
            {                    
                if(++times_connectPhoXiControl % 20 == 0)
                    RCLCPP_INFO(_node->get_logger(), "%s try to connect PhoXiControl ...", GetLocalTime());
                if(++times_connectPhoXiControl % 300 == 0)
                    throw std::runtime_error("Error in Function GetAvailableDevices(): failed to connect PhoXiControl.");
                std::this_thread::sleep_for(100ms);
            }
            RCLCPP_INFO(_node->get_logger(), "%s connect PhoXiControl successfully.", GetLocalTime());
           
            //get device list
            unsigned long times_getdevice = 0;
            while(DeviceList.size() < 2)
            {                    
                DeviceList = Factory.GetDeviceList();
                if(++times_getdevice % 20 == 0)
                    RCLCPP_INFO(_node->get_logger(), "%s try to find device ...", GetLocalTime());
                if(++times_connectPhoXiControl % 300 == 0)
                    throw std::runtime_error("Error in Function GetAvailableDevices(): failed to get available device.");
                std::this_thread::sleep_for(100ms);
            }
            RCLCPP_INFO(_node->get_logger(), "%s get available devices.", GetLocalTime());
        }

        bool IsDeviceConnected()
        {
            if (!PhoXiDevice || !PhoXiDevice->isConnected())
                return false;
            return true;
        }

        void ConnectPhoXiDevice()
        {
            unsigned long times_connectDevice = 0;
            while(!IsDeviceConnected())
            {      
                std::cout << "connectionMethod: " << connectionMethod << std::endl;
                std::cout << "width: " << width << std::endl;
                if( connectionMethod.compare("HWIdentification") == 0)
                    ConnectPhoXiDeviceByHWIdentification();                     
                else
                    throw std::runtime_error("Error in Function ConnectPhoXiDevice(): Wrong ConnectionMethod!\n");                     

                if(++times_connectDevice % 20 == 0)
                    RCLCPP_INFO(_node->get_logger(), "%s try to connect device ...", GetLocalTime());
                if(++times_connectDevice % 300 == 0)
                {
                    std::string error = "Error in Function ConnectPhoXiDevice(): failed to connect device with HWIdentification:" +  HWIdentification;
                    throw std::runtime_error(error);
                }
                    
                std::this_thread::sleep_for(100ms);
            }
            RCLCPP_INFO(_node->get_logger(), "%s connection to the device %s with Hardware Identification %s.", 
                    GetLocalTime(), 
                    std::string(PhoXiDevice->GetType()).c_str(),
                    std::string(PhoXiDevice->HardwareIdentification).c_str());
        }       

        void ConnectPhoXiDeviceByHWIdentification()
        {            
            pho::api::PhoXiTimeout Timeout = pho::api::PhoXiTimeout::ZeroTimeout;
            PhoXiDevice = Factory.CreateAndConnect(HWIdentification, Timeout);
        }

        void StartAcquisition()
        {
            if (!PhoXiDevice->isAcquiring())
            {
                if (!PhoXiDevice->StartAcquisition())
                    throw std::runtime_error("Error in Function StartAcquisition(): failed to start acquisition.");
                RCLCPP_INFO(_node->get_logger(), "%s start acquisition.", GetLocalTime());
            }
        }

        void StopAcquisition()
        {
            if (PhoXiDevice->isAcquiring())
            {
                if (!PhoXiDevice->StopAcquisition())
                    throw std::runtime_error("Error in Function StopAcquisition(): failed to stop acquisition.");
                RCLCPP_INFO(_node->get_logger(), "%s stop acquisition.", GetLocalTime());
            }
        }

        /**** change setting ****/
        void ChangeSettings()
        {
            if(!IsDeviceConnected())
                throw std::runtime_error("Error in Function ChangeSettings(): device is not connected.");

            SetCapturingMode();

            SetTriggerMode();

            SetTimeOut();

            ChangeCapturingSettings();

            ChangeProcessingSettings();

            ChangeCoordinatesSettings();

            ChangeFrameOutputSettings();
        }

        void SetCapturingMode()
        {
            if (!PhoXiDevice || !PhoXiDevice->isConnected())
                throw std::runtime_error("Error in Function SetCapturingMode(): Device is not connected!");

            StopAcquisition();

            if (!PhoXiDevice->CapturingMode.isEnabled() || 
                !PhoXiDevice->CapturingMode.CanSet() || 
                !PhoXiDevice->CapturingMode.CanGet())
                throw std::runtime_error("Error in Function SetCapturingMode(): CapturingMode used in are not supported by the Device Hardware, or are Read only on the specific device!");

            pho::api::PhoXiCapturingMode newPhoXiCapturingMode;
            newPhoXiCapturingMode.Resolution.Width = width;
            newPhoXiCapturingMode.Resolution.Height = height;

            PhoXiDevice -> CapturingMode = newPhoXiCapturingMode;
            if (!PhoXiDevice->CapturingMode.isLastOperationSuccessful())
            {
                std::string errorMessage = "Error in Function ChangeFrameOutputSettings()" + PhoXiDevice->OutputSettings.GetLastErrorMessage();
                throw std::runtime_error(errorMessage);
            }                
            // else
                // RCLCPP_INFO(_node->get_logger(), "CapturingMode has been changed successfully.");
        }

        void SetTriggerMode()
        {
            if (!PhoXiDevice || !PhoXiDevice->isConnected())
                throw std::runtime_error("Error in Function SetTriggerMode(): Device is not connected!");

            StopAcquisition();

            if (!PhoXiDevice->TriggerMode.isEnabled() || 
                !PhoXiDevice->TriggerMode.CanSet() || 
                !PhoXiDevice->TriggerMode.CanGet())
                throw std::runtime_error("Error in Function SetTriggerMode(): TriggerMode used in are not supported by the Device Hardware, or are Read only on the specific device!");

            if(triggerMode.compare("Freerun") == 0)
                PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Freerun;
            else if(triggerMode.compare("Software") == 0)
                PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
            else if(triggerMode.compare("Hardware") == 0)
                PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Hardware;
            else if(triggerMode.compare("NoValue") == 0)
                PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::NoValue;
            else
                throw std::runtime_error("Error in Function SetTriggerMode(): TriggerMode is invalid!");

            if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful())
            {
                std::string errorMessage = "Error in Function SetTriggerMode(): " + PhoXiDevice->OutputSettings.GetLastErrorMessage();
                throw std::runtime_error(errorMessage);
            }
            // else
                // RCLCPP_INFO(_node->get_logger(), "TriggerMode has been changed successfully.");
        }

        void SetTimeOut()
        {
            if (!PhoXiDevice || !PhoXiDevice->isConnected())
                throw std::runtime_error("Error in Function SetTimeOut(): Device is not connected!");

            StopAcquisition();

            if (!PhoXiDevice->Timeout.isEnabled() || 
                !PhoXiDevice->Timeout.CanSet() || 
                !PhoXiDevice->Timeout.CanGet())
                throw std::runtime_error("Error in Function SetTimeOut(): Timeout used in are not supported by the Device Hardware, or are Read only on the specific device!");

            // PhoXiDevice->Timeout = timeOut;

            if(timeOut.compare("ZeroTimeout") == 0)
                PhoXiDevice->Timeout = pho::api::PhoXiTimeout::ZeroTimeout;
            else if(timeOut.compare("Infinity") == 0)
                PhoXiDevice->Timeout = pho::api::PhoXiTimeout::Infinity;
            else if(timeOut.compare("LastStored") == 0)
                PhoXiDevice->Timeout = pho::api::PhoXiTimeout::LastStored;
            else if(timeOut.compare("Default") == 0)
                PhoXiDevice->Timeout = pho::api::PhoXiTimeout::Default;
            else
                throw std::runtime_error("Error in Function SetTimeOut(): Timeout is invalid!");

            if (!PhoXiDevice->Timeout.isLastOperationSuccessful())
            {
                std::string errorMessage = "Error in Function SetTimeOut(): " + PhoXiDevice->OutputSettings.GetLastErrorMessage();
                throw std::runtime_error(errorMessage);
            }
            // else
                // RCLCPP_INFO(_node->get_logger(), "Timeout has been changed successfully.");
        }

        void ChangeCapturingSettings()
        {
            if (!PhoXiDevice || !PhoXiDevice->isConnected())
                throw std::runtime_error("Error in Function ChangeCapturingSettings(): Device is not connected!");

            StopAcquisition();

            if (!PhoXiDevice->CapturingSettings.isEnabled() || 
                !PhoXiDevice->CapturingSettings.CanSet() || 
                !PhoXiDevice->CapturingSettings.CanGet())
                throw std::runtime_error("Error in Function ChangeCapturingSettings(): CapturingSettings used in are not supported by the Device Hardware, or are Read only on the specific device!");

            pho::api::PhoXiCapturingSettings NewCapturingSettings;
            NewCapturingSettings.ShutterMultiplier = shutterMultiplier;
            NewCapturingSettings.ScanMultiplier = scanMultiplier;
            NewCapturingSettings.CameraOnlyMode = cameraOnlyMode;
            NewCapturingSettings.AmbientLightSuppression = ambientLightSuppression;
            NewCapturingSettings.MaximumFPS = maximumFPS;
            NewCapturingSettings.SinglePatternExposure = singlePatternExposure;
            NewCapturingSettings.CodingStrategy =codingStrategy;                        //enum Value { NoValue = 0, Normal = 1, Interreflections = 2 }
            NewCapturingSettings.CodingQuality = codingQuality;                          //enum Value { NoValue = 0, Fast = 1, High = 2, Ultra = 3 }
            NewCapturingSettings.TextureSource = textureSource;                          //enum Value { NoValue = 0, Computed = 1, LED = 2, Laser = 3, Focus = 4, Color = 5 }
            NewCapturingSettings.LaserPower = laserPower;
            NewCapturingSettings.LEDPower = lEDPower;
            NewCapturingSettings.ProjectionOffsetLeft = projectionOffsetLeft;
            NewCapturingSettings.ProjectionOffsetRight = projectionOffsetRight;
            PhoXiDevice -> CapturingSettings = NewCapturingSettings;

            if (!PhoXiDevice->CapturingSettings.isLastOperationSuccessful())
            {
                std::string errorMessage = "Error in Function ChangeCapturingSettings(): " + PhoXiDevice->OutputSettings.GetLastErrorMessage();
                throw std::runtime_error(errorMessage);
            }
            // else
                // RCLCPP_INFO(_node->get_logger(), "CapturingSettings has been changed successfully.");
        }

        void ChangeProcessingSettings()
        {
            if (!PhoXiDevice || !PhoXiDevice->isConnected())
                throw std::runtime_error("Error in Function ChangeProcessingSettings(): Device is not connected!");

            StopAcquisition();

            if (!PhoXiDevice->ProcessingSettings.isEnabled() || 
                !PhoXiDevice->ProcessingSettings.CanSet() || 
                !PhoXiDevice->ProcessingSettings.CanGet())
                throw std::runtime_error("Error in Function ChangeProcessingSettings(): ProcessingSettings used in are not supported by the Device Hardware, or are Read only on the specific device!");

            pho::api::PhoXiProcessingSettings NewProcessingSettings;

            if(roiSpace.compare("CameraSpace") == 0)
            {
                NewProcessingSettings.ROI3D.CameraSpace.min.x = minX;
                NewProcessingSettings.ROI3D.CameraSpace.min.y = minY;
                NewProcessingSettings.ROI3D.CameraSpace.min.z = minZ;
                NewProcessingSettings.ROI3D.CameraSpace.max.x = maxX;
                NewProcessingSettings.ROI3D.CameraSpace.max.y = maxY;
                NewProcessingSettings.ROI3D.CameraSpace.max.z = maxZ;
            }
            else if(roiSpace.compare("PointCloudSpace") == 0)
            {
                NewProcessingSettings.ROI3D.PointCloudSpace.min.x = minX;
                NewProcessingSettings.ROI3D.PointCloudSpace.min.y = minY;
                NewProcessingSettings.ROI3D.PointCloudSpace.min.z = minZ;
                NewProcessingSettings.ROI3D.PointCloudSpace.max.x = maxX;
                NewProcessingSettings.ROI3D.PointCloudSpace.max.y = maxY;
                NewProcessingSettings.ROI3D.PointCloudSpace.max.z = maxZ;       
            }

            //MaxCameraAngle values: 0-90
            NewProcessingSettings.NormalAngle.MaxCameraAngle = maxCameraAngle;

            //MaxProjectionAngle values: 0-90
            NewProcessingSettings.NormalAngle.MaxProjectorAngle =maxProjectorAngle;

            //MinHalfwayAngle values: 0-90
            NewProcessingSettings.NormalAngle.MinHalfwayAngle = minHalfwayAngle;

            //MaxHalfwayAngle values: 0-90
            NewProcessingSettings.NormalAngle.MaxHalfwayAngle = maxHalfwayAngle;

            //MaxInaccuracy(Confidence) values: 0-100
            NewProcessingSettings.Confidence = confidence;

            //CalibrationVolumeCut values: 0 / 1 or false / true (0 is OFF, 1 is ON)
            NewProcessingSettings.InterreflectionsFiltering = calibrationVolumeOnly;
            
            //SurfaceSmoothness values: Sharp / Normal / Smooth
            NewProcessingSettings.SurfaceSmoothness =surfaceSmothness;

            //NormalsEstimationRadius values: 1-4
            NewProcessingSettings.NormalsEstimationRadius = normalsEstimationRadius;

            //InterreflectionsFiltering values: 0 / 1 or false / true (0 is OFF, 1 is ON)
            NewProcessingSettings.InterreflectionsFiltering = interreflectionsFiltering;

            //InterreflectionFilterStrength values: 0.01-0.99
            NewProcessingSettings.InterreflectionFilterStrength = interreflectionFilterStrength;

            PhoXiDevice->ProcessingSettings = NewProcessingSettings;
            //Check if the CurrentProcessingSettings have been retrieved succesfully
            if (!PhoXiDevice->ProcessingSettings.isLastOperationSuccessful())
            {
                std::string errorMessage = "Error in Function ChangeProcessingSettings(): " + PhoXiDevice->OutputSettings.GetLastErrorMessage();
                throw std::runtime_error(errorMessage);
            }                
            // else
                // RCLCPP_INFO(_node->get_logger(), "ProcessingSettings has been changed successfully.");
        }

        void ChangeCoordinatesSettings()
        {
            if (!PhoXiDevice || !PhoXiDevice->isConnected())
                throw std::runtime_error("Error in Function ChangeCoordinatesSettings(): Device is not connected!");

            StopAcquisition();

            if (!PhoXiDevice->CoordinatesSettings.isEnabled() || 
                !PhoXiDevice->CoordinatesSettings.CanSet() || 
                !PhoXiDevice->CoordinatesSettings.CanGet())
                throw std::runtime_error("Error in Function ChangeCoordinatesSettings(): CoordinatesSettings used in are not supported by the Device Hardware, or are Read only on the specific device!");

            pho::api::PhoXiCoordinatesSettings NewCoordinatesSettings;

            {
                pho::api::PhoXiCoordinateTransformation _customTransformation;
                auto _rotationCustom = rotationCustom;
                for(int i = 0; i < 3 ; ++i)
                {
                    for(int j = 0; j < 3; ++j)
                    {
                        _customTransformation.Rotation.At(i, j) = _rotationCustom[i*2+j];
                    }
                }

                auto _translationCustom = translationCustom;
                _customTransformation.Translation.x = _translationCustom[0];
                _customTransformation.Translation.y = _translationCustom[1];
                _customTransformation.Translation.z = _translationCustom[2];

                //CustomSpace Transformation
                PhoXiDevice->CoordinatesSettings->CustomTransformation = _customTransformation;        
            }

            {
                pho::api::PhoXiCoordinateTransformation _robotTransformation;
                auto _rotationRobot = rotationRobot;
                for(int i = 0; i < 3 ; ++i)
                {
                    for(int j = 0; j < 3; ++j)
                    {
                        _robotTransformation.Rotation.At(i, j) = _rotationRobot[i*2+j];
                    }
                }

                auto _translationRobot = translationRobot;
                _robotTransformation.Translation.x = _translationRobot[0];
                _robotTransformation.Translation.y = _translationRobot[1];
                _robotTransformation.Translation.z = _translationRobot[2];

                //CustomSpace Transformation
                PhoXiDevice->CoordinatesSettings->RobotTransformation = _robotTransformation;        
            }

            //Recognize Markers values: 0 / 1 or false / true (0 is OFF, 1 is ON)
            PhoXiDevice->CoordinatesSettings->RecognizeMarkers = recognizeMarkers;

            //Pattern Scale values: 0.0 - 1.0 (scale 1.0 x 1.0 is normal size)
            pho::api::PhoXiSize_64f scale;
            scale.Width = markerScaleW;
            scale.Height = markerScaleH;
            PhoXiDevice->CoordinatesSettings->MarkersSettings.MarkerScale = scale;

            PhoXiDevice->CoordinatesSettings->MarkersSettings.InvertedMarkers = invertedMarkers;

            PhoXiDevice->CoordinatesSettings->CoordinateSpace = coordinateSpace;

            PhoXiDevice->CoordinatesSettings = NewCoordinatesSettings;

            //Check if the ChangedProcessingSettings have been retrieved succesfully
            if (!PhoXiDevice->CoordinatesSettings.isLastOperationSuccessful()) 
            {
                std::string errorMessage = "Error in Function ChangeCoordinatesSettings(): " + PhoXiDevice->OutputSettings.GetLastErrorMessage();
                throw std::runtime_error(errorMessage);
            }                
            // else
                // RCLCPP_INFO(_node->get_logger(), "CoordinatesSettings has been changed successfully.");
        }

        void ChangeFrameOutputSettings()
        {
            if (!PhoXiDevice || !PhoXiDevice->isConnected())
                throw std::runtime_error("Error in Function ChangeFrameOutputSettings(): Device is not connected!");

            StopAcquisition();

            if (!PhoXiDevice->OutputSettings.isEnabled() || 
                !PhoXiDevice->OutputSettings.CanSet() || 
                !PhoXiDevice->OutputSettings.CanGet())
                throw std::runtime_error("Error in Function ChangeFrameOutputSettings(): OutputSettings used in are not supported by the Device Hardware, or are Read only on the specific device!");

            //Get the current Output configuration
            pho::api::FrameOutputSettings NewOutputSettings;
            if(cameraOnlyMode)
            {
                NewOutputSettings.SendPointCloud = false;
                NewOutputSettings.SendNormalMap = false;
            }
            else
            {
                NewOutputSettings.SendPointCloud = true;
                NewOutputSettings.SendNormalMap = true;
            }
            NewOutputSettings.SendDepthMap = false;
            NewOutputSettings.SendConfidenceMap = false;
            NewOutputSettings.SendTexture = true;
            NewOutputSettings.SendEventMap = false;

            PhoXiDevice->OutputSettings = NewOutputSettings;

            if (!PhoXiDevice->OutputSettings.isLastOperationSuccessful())
            {
                std::string errorMessage = "Error in Function ChangeFrameOutputSettings(): " + PhoXiDevice->OutputSettings.GetLastErrorMessage();
                throw std::runtime_error(errorMessage);
            }
            // else
                // RCLCPP_INFO(_node->get_logger(), "OutputSettings has been changed successfully.");
        }            

        /**** grab ****/
        void Grab(
            const long& id, 
            const std::string& engineering, 
            const std::string& option,
            const float& angle, 
            const int& frameID) try
        {
            if (!IsDeviceConnected())
                throw std::runtime_error("Error in Function Grab(): device is not connected.");

            StartAcquisition();

            int ClearedFrames = PhoXiDevice->ClearBuffer();
            RCLCPP_INFO(_node->get_logger(), " %s %d were cleared from the cyclic buffer", GetLocalTime(), ClearedFrames);

            int FrameID = PhoXiDevice->TriggerFrame();
            if (FrameID < 0)
                throw std::runtime_error("Error in Function Grab(): trigger was unsuccessful!");                 
            else
                RCLCPP_INFO(_node->get_logger(), "trigger was successful, Frame Id: %d", FrameID);                

            SampleFrame = PhoXiDevice->GetSpecificFrame(FrameID, pho::api::PhoXiTimeout::Infinity/*, You can specify Timeout here - default is the Timeout stored in Timeout Feature -> Infinity by default*/);

            if (!SampleFrame || SampleFrame->Empty())
                throw std::runtime_error("Error in Function Grab(): SampleFrame is empty!");

            cv::Mat texture;
            // SaveFrame();

            if(option == "EmbeddedParts" || option == "DefectDetection")
            {                    
                ConvertTextureToOpenCV(SampleFrame,texture); 
                _node -> _PubImage(texture, id, engineering, option, angle, frameID);                
            }
            else if(engineering == "Masonry")
            {
                if(option == "WallVertical" || option == "SideWallFlatness")
                {
                    ConvertTextureToOpenCV(SampleFrame,texture); 
                    _node -> _PubImage(texture, id, engineering, option, angle, frameID);
                    std::this_thread::sleep_for(2000ms);
                    _node -> _PubPointCloud(SampleFrame, id, engineering, option, angle, frameID);
                }
            }
            else
            {
                _node->_PubPointCloud(SampleFrame, id, engineering, option, angle, frameID);
            }



            // ConvertTextureToOpenCV(SampleFrame,texture);

            // time_t now = time(0);
            // std::tm* localTime = std::localtime(&now);
            // std::string time_str = "[" + std::to_string(1900 + localTime->tm_year) + "-" 
            //     + std::to_string(1 + localTime->tm_mon) + "-"
            //     + std::to_string(localTime->tm_mday) + " "
            //     + std::to_string(localTime->tm_hour) + ":"
            //     + std::to_string(localTime->tm_min) + ":"
            //     + std::to_string(localTime->tm_sec) + "]";

            // std::string name = "/home/ubuntu/tmp/image_";
            // // std::string time = GetLocalTime();
            // std::string type = ".png";
            // name = name + time_str + type;
            // cv::imwrite(name, texture);
        }
        catch (const std::exception & e) 
        {
            RCLCPP_INFO(_node->get_logger(), "%s Exception: %s", GetLocalTime(), e.what());
            rclcpp::shutdown();
        } 
        catch (...) 
        {
            RCLCPP_INFO(_node->get_logger(), "%s Exception: unknown", GetLocalTime());
            rclcpp::shutdown();
        }

        void SaveFrame()
        {
            const auto outputFolder_delimiter = outputFolder + DELIMITER;
            const auto sampleFramePly = outputFolder_delimiter + projectName + std::to_string((SampleFrame->Info).FrameIndex) + ".ply";
            if (SampleFrame->SaveAsPly(sampleFramePly, true, true))
            {
                std::cout << "Save sample frame as PLY to: " << sampleFramePly << std::endl << std::endl;
            }
            else
            {    
                std::cout << "Could not save sample frame as PLY to " << sampleFramePly << " !" << std::endl << std::endl;
            }
        }

        // int Write(const std::string& fileName, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
        // {
        //     std::ofstream ofile(fileName,std::ofstream::out);
        //     if(!ofile.is_open())
        //         return -1;
            
        //     ofile << std::fixed;
        //     for (const auto & p : *cloud) 
        //     {
        //         ofile 
        //             << p.x  << ' ' 
        //             << p.y  << ' ' 
        //             << p.z  << ' ' 
        //             << p.intensity << '\n';
        //     }

        //     ofile.close();

        //     return 0;
        // }

        // void SaveFrameToTXT()
        // {
        //     const auto outputFolder_delimiter = outputFolder + DELIMITER;
        //     const auto sampleFrameTxt = outputFolder_delimiter + projectName + std::to_string((SampleFrame->Info).FrameIndex) + ".txt";

        //     Write(sampleFrameTxt, )

        // }


        void ConvertTextureToOpenCV(const pho::api::PFrame &frame,cv::Mat& textureMat08bit)
        { 
            cv::Mat textureMat32bit; 
            if (!frame->Texture.Empty()) 
            { 
                if (frame->Texture.ConvertTo(textureMat32bit))
                { 
                    double minVal;
                    double maxVal;
                    cv::Point minPos;
                    cv::Point maxPos;
                    cv::minMaxLoc(textureMat32bit,&minVal,&maxVal,&minPos,&maxPos);

                    textureMat32bit.convertTo(textureMat08bit, CV_8UC1, 255./maxVal, 0); 

                } 
            } 
        }

    private:
        /****params****/

        //resolution      
        int width;
        int height;

        std::string triggerMode;
        std::string timeOut;

        //captruing settings
        int shutterMultiplier;
        int scanMultiplier;
        bool cameraOnlyMode;
        bool ambientLightSuppression;
        double maximumFPS;
        double singlePatternExposure;
        std::string codingStrategy;
        std::string codingQuality;
        std::string textureSource;
        int laserPower;
        int lEDPower;
        int projectionOffsetLeft;
        int projectionOffsetRight;

        //processing settings
        double confidence;
        std::string roiSpace;
        double minX;
        double minY;
        double minZ;
        double maxX;
        double maxY;
        double maxZ;
        double maxCameraAngle;
        double maxProjectorAngle;
        double minHalfwayAngle;
        double maxHalfwayAngle;
        std::string surfaceSmothness;
        bool calibrationVolumeOnly;
        int normalsEstimationRadius;
        bool interreflectionsFiltering;
        double interreflectionFilterStrength;

        //coordinate settings
        bool invertedMarkers;
        double markerScaleW;
        double markerScaleH;
        std::string coordinateSpace;
        std::vector<double> rotationCustom;
        std::vector<double> translationCustom;
        std::vector<double> rotationRobot;
        std::vector<double> translationRobot;
        bool recognizeMarkers;

        //others
        std::string connectionMethod;
        std::string HWIdentification;

        std::string fileCameraFolder;
        std::string outputFolder;

        std::string projectName;

        bool saveFrame;

        /**** others ****/
        pho::api::PhoXiFactory Factory;
        pho::api::PPhoXi PhoXiDevice;
        pho::api::PFrame SampleFrame;
        std::vector <pho::api::PhoXiDeviceInformation> DeviceList;

        PhoXiControl* _node;
};

/**** PhoXiControl ****/
PhoXiControl::PhoXiControl(const rclcpp::NodeOptions& option) : rclcpp::Node("phoxi_control_node", option)
{
    _initThread = std::thread(&PhoXiControl::_Init, this);
}

PhoXiControl::~PhoXiControl() try
{
    _initThread.join();

    _impl.reset();

    _srvGrabPointCloud.reset();
    _srvGrabImage.reset();
    _srvStart.reset();
    _srvStop.reset();

    _pubPointCloud.reset();
    _pubImageEmbeddedParts.reset();
    _pubImageDefectDetection.reset();
    _pubImageMortarJoint.reset();
    _pubStatus.reset();

    RCLCPP_INFO(this->get_logger(),"%s PhoXiControl destroyed successfully", GetLocalTime());
}
catch (const std::exception & e) 
{
    RCLCPP_INFO(this->get_logger(), "%s Exception in destruction: %s",  GetLocalTime(), e.what());
    rclcpp::shutdown();
} 
catch (...) 
{
    RCLCPP_INFO(this->get_logger(), "%s Exception in destruction: unknown", GetLocalTime());
    rclcpp::shutdown();
}

void PhoXiControl::_Init() try
{
    _status = -1;

    _srvStart = this->create_service<shared_interfaces::srv::TriggerOp>(_srvStartName, std::bind(&PhoXiControl::_SrvStart, this, std::placeholders::_1, std::placeholders::_2));
    _srvStop = this->create_service<shared_interfaces::srv::TriggerOp>(_srvStopName, std::bind(&PhoXiControl::_SrvStop, this, std::placeholders::_1, std::placeholders::_2));

    _srvGrabPointCloud = this->create_service<shared_interfaces::srv::TriggerOp>(_srvGrabPointCloudName, std::bind(&PhoXiControl::_SrvGrabPointCloud, this, std::placeholders::_1, std::placeholders::_2));
    _srvGrabImage = this->create_service<shared_interfaces::srv::TriggerOp>(_srvGrabImageName, std::bind(&PhoXiControl::_SrvGrabImage, this, std::placeholders::_1, std::placeholders::_2));

    _pubPointCloud = this->create_publisher<shared_interfaces::msg::PointCloudC>(_pubPointCloudName, 10);
    _pubImageEmbeddedParts = this->create_publisher<shared_interfaces::msg::ImageC>(_pubImageEmbeddedPartsName, 2);
    _pubImageDefectDetection = this->create_publisher<shared_interfaces::msg::ImageC>(_pubImageDefectDetectionName, 2);
    _pubImageMortarJoint = this->create_publisher<shared_interfaces::msg::ImageC>(_pubImageMortarJointName, 2);
    _pubStatus = this->create_publisher<std_msgs::msg::String>(_pubStatusName, 10);

    _impl = std::make_unique<_Impl>(this);

    _status = 0;
    
    RCLCPP_INFO(this->get_logger(), "PhoXiControl initialized successfully");
}
catch (const std::exception & e) 
{
    RCLCPP_INFO(this->get_logger(), "Exception in initializer: %s", e.what());
    rclcpp::shutdown();
} 
catch (...) 
{
    RCLCPP_INFO(this->get_logger(), "Exception in initializer: unknown");
    rclcpp::shutdown();
}

void PhoXiControl::_InitializeParameters()
{
    return;
}

void PhoXiControl::_UpdateParameters()
{
    return;
}

void PhoXiControl::_SrvStart(
    const std::shared_ptr<shared_interfaces::srv::TriggerOp::Request>, 
    std::shared_ptr<shared_interfaces::srv::TriggerOp::Response> response)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"%s initialize and receive a service call: start.",GetLocalTime());
        return;
    }

    response->success = false;
    response->message = "Fail: photoneo start";

    _impl->StartAcquisition();

    response->success = true;
    response->message = "Success: photoneo start";
}

void PhoXiControl::_SrvStop(
    const std::shared_ptr<shared_interfaces::srv::TriggerOp::Request>, 
    std::shared_ptr<shared_interfaces::srv::TriggerOp::Response> response)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"%s initialize and receive a service call: stop.",GetLocalTime());
        return;
    }
    
    response->success = false;
    response->message = "Fail: photoneo stop";

    _impl->StopAcquisition();

    response->success = true;
    response->message = "Success: photoneo stop";
}

void PhoXiControl::_SrvGrabPointCloud(
    const std::shared_ptr<shared_interfaces::srv::TriggerOp::Request> request, 
    std::shared_ptr<shared_interfaces::srv::TriggerOp::Response> response)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"%s initialize and receive a service call: grab point cloud.",GetLocalTime());
        return;
    }

    response->success = false;
    response->message = "Fail: photoneo grab";

    const long id = request -> id;
    const std::string engineering = request -> engineering;
    const std::string option = request -> option;
    const float angle = request -> angle;
    const int frameID = request -> frame_id;
    _impl->Grab(id, engineering, option, angle,frameID);

    response->success = true;
    response->message = "Success: photoneo grab";
}


void PhoXiControl::_SrvGrabImage(
    const std::shared_ptr<shared_interfaces::srv::TriggerOp::Request> request,
    std::shared_ptr<shared_interfaces::srv::TriggerOp::Response> response)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"%s initialize and receive a service call: grab image.",GetLocalTime());
        return;
    }

    response->success = false;
    response->message = "Fail: photoneo image grab";

    const long id = request -> id;
    const std::string engineering = request -> engineering;
    const std::string option = request -> option;
    const float angle = request -> angle;
    const int frameID = request -> frame_id;
    _impl->Grab(id, engineering, option, angle,frameID);

    response->success = true;
    response->message = "Success: photoneo image grab";
}

void PhoXiControl::_PubPointCloud(
    const pho::api::PFrame &frame, 
    const long& id,
    const std::string& engineering, 
    const std::string& option,
    const float& angle,
    const int& frameID)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"%s initialize and publish a point cloud.",GetLocalTime());
        return;
    }
    auto PointCloudCPtr = std::make_unique<shared_interfaces::msg::PointCloudC>(); 

    pho::api::PhoXiSize Resolution = frame->GetResolution();
    if (Resolution ==  pho::api::PhoXiSize(0, 0) || frame->Texture.Empty())
        return;

    PointCloudCPtr -> id = id;
    PointCloudCPtr -> engineering = engineering;
    PointCloudCPtr -> option = option;
    PointCloudCPtr -> angle = angle;
    PointCloudCPtr -> frame_id = frameID;
    PointCloudCPtr -> width = Resolution.Width;
    PointCloudCPtr -> height = Resolution.Height;

    PointCloudCPtr -> x.reserve(Resolution.Width * Resolution.Height);      
    PointCloudCPtr -> y.reserve(Resolution.Width * Resolution.Height);   
    PointCloudCPtr -> z.reserve(Resolution.Width * Resolution.Height);   
    PointCloudCPtr -> intensity.reserve(Resolution.Width * Resolution.Height);             

    for(size_t i = 0; i < PointCloudCPtr -> height; i++)//row
    {
        for(size_t j = 0; j < PointCloudCPtr -> width; j++)//col
        {
            auto& s = frame->PointCloud.At(i,j);
            auto& t = frame->Texture.At(i,j);

            (PointCloudCPtr -> x).push_back(float(s.x));
            (PointCloudCPtr -> y).push_back(float(s.y));
            (PointCloudCPtr -> z).push_back(float(s.z));
            (PointCloudCPtr -> intensity).push_back(t);
        }
    }    

    _pubPointCloud->publish(std::move(PointCloudCPtr));   
    RCLCPP_INFO(this->get_logger(),"%s, publish one point cloud.", GetLocalTime()); 
}

void PhoXiControl::_PubImage(
    const cv::Mat& image,
    const long& id,
    const std::string& engineering,
    const std::string& option,
    const float& angle,
    const int& frameID)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"%s initialize and publish a image.",GetLocalTime());
        return;
    }

    auto imagePtr = std::make_unique<shared_interfaces::msg::ImageC>();
    imagePtr->id = id;
    imagePtr->engineering = engineering;
    imagePtr->option = option;
    imagePtr->angle = angle;
    imagePtr->frame_id = frameID;

    

    imagePtr->image.header.stamp = this->now();
    auto h = imagePtr->image.height = image.rows;
    auto w = imagePtr->image.width = image.cols;
    imagePtr->image.encoding = "mono8";
    imagePtr->image.is_bigendian = false;
    imagePtr->image.step  = image.cols;
    imagePtr->image.data.resize(h * w);
    std::cout << "********************image.height: " << imagePtr->image.height << std::endl;
    std::cout << "********************image.width: " << imagePtr->image.width << std::endl;

    for (decltype(h) r = 0; r < h; ++r) 
    {
        auto headI = (unsigned char *)(image.data) + w * r;  // NOLINT
        auto headP = (unsigned char *)(imagePtr->image.data.data()) + w * r;    // NOLINT
        memcpy(headP, headI, w);
    }  
    if(option == "DefectDetection")
    {
        _pubImageDefectDetection->publish(std::move(imagePtr));
        RCLCPP_INFO(this->get_logger(),"%s, publish one frame image to ~/image_defect_detection.", GetLocalTime()); 
    }
    else if(option == "EmbeddedParts")
    {
        _pubImageEmbeddedParts->publish(std::move(imagePtr));
        RCLCPP_INFO(this->get_logger(),"%s, publish one frame image to ~/image_embedded_parts.", GetLocalTime()); 
    }
    else if(engineering == "Masonry")
    {
        _pubImageMortarJoint->publish(std::move(imagePtr));
        RCLCPP_INFO(this->get_logger(),"%s, publish one frame image to ~/image_mortar_joint.", GetLocalTime()); 
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "%s Error in _PubImage(): option is invalid.",GetLocalTime());
    }
}

void PhoXiControl::_PubStatus(const std::string& status)
{
    return;
}

}//phoxi_control


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(phoxi_control::PhoXiControl)

