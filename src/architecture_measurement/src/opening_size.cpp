#include "architecture_measurement/opening_size.hpp"

namespace am
{

OpeningSize::OpeningSize(const rclcpp::NodeOptions& option) : rclcpp::Node("surface_flatness_node", option)
{
    _initThread = std::thread(&OpeningSize::Init, this);
}

OpeningSize::~OpeningSize() try
{
    _status = -1;
    _initThread.join();

    _subPointCloud.reset();
    _subInclinometer.reset();
    _pubResult.reset();
    _pubStatus.reset();

    RCLCPP_INFO(this->get_logger(),"OpeningSize destroyed successfully.");
}
catch(const std::exception& e)
{
    RCLCPP_INFO(this->get_logger(), "Exception in OpeningSize destruction: %s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_INFO(this->get_logger(),"Exception in OpeningSize destruction: unknown");
    rclcpp::shutdown();
}

void OpeningSize::Init() try
{
    _status = -1;

    InitializeParameters();
    UpdateParameters();

    _subPointCloud = this->create_subscription<shared_interfaces::msg::PointCloudC>(
        _subPointCloudName, 
        1, 
        std::bind(&OpeningSize::SubPointCloud, this, std::placeholders::_1));

    _subInclinometer = this->create_subscription<shared_interfaces::msg::Inclinometer>(
        _subInclinometerName, 
        5, 
        std::bind(&OpeningSize::SubInclinometer, this, std::placeholders::_1)
    );

    _pubResult = this->create_publisher<shared_interfaces::msg::MeasurementResult>(_pubResultName, 1);

    _pubStatus = this->create_publisher<std_msgs::msg::String>(_pubStatusName, 1);

    _status = 0;

    RCLCPP_INFO(this->get_logger(), "OpeningSize initialized successfully.");
}
catch(const std::exception& e)
{
    RCLCPP_INFO(this->get_logger(), "Exception in OpeningSize initializer: %s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_INFO(this->get_logger(),"Exception in OpeningSize initializer: unknown");
    rclcpp::shutdown();
}

void OpeningSize::InitializeParameters()
{
    this->declare_parameter<std::vector<std::string>>("engineering", {"Normal"});
    this->declare_parameter<std::vector<std::string>>("option",{"StoreyHeight"});
    this->declare_parameter<std::vector<double>>("I_R_C",{0.});
    this->declare_parameter<std::string>("fileName","../../result/pointCloud.txt");
}

void OpeningSize::UpdateParameters()
{
    std::vector<double> I_R_C_vector;     

    this->get_parameter("engineering", _engineeringVec);
    this->get_parameter("option", _optionVec); 
    this->get_parameter("I_R_C",I_R_C_vector);
    this->get_parameter("fileName",_fileName);

    double* I_R_C_vector_ptr = I_R_C_vector.data();
    Eigen::Matrix3d I_R_C_tmp = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> >(I_R_C_vector_ptr);
    _I_R_C = I_R_C_tmp.cast<float>();

}

void OpeningSize::SubPointCloud(shared_interfaces::msg::PointCloudC::UniquePtr pointCloudCPtr)
{
    try
    {    
        if(_status < 0)
        {
            RCLCPP_INFO(this->get_logger(),"initialize and subscribe a point cloud.");
            return;
        }

        if(!(pointCloudCPtr -> option == "StoreyHeight"))
            return;

        RCLCPP_INFO(this->get_logger(),"subscribe a point cloud.");

        const long id = pointCloudCPtr->id;
        const std::string engineering = pointCloudCPtr->engineering.c_str();
        const std::string option = pointCloudCPtr->option.c_str();
        const float angle = pointCloudCPtr->angle;
        const int frameID = pointCloudCPtr->frame_id;

        std::cout << "id: " << id << std::endl;
        std::cout << "engineering: " << engineering << std::endl;
        std::cout << "option: " << option << std::endl;
        std::cout << "angle: " << angle <<std::endl;
        std::cout << "frameID: " << frameID << std::endl;

        {
            auto iter = std::find_if(_engineeringVec.begin(), _engineeringVec.end(), [&engineering](const std::string& engineeringVec)
            {
                return EndsWith(engineeringVec, engineering);
            });
            if(iter == _engineeringVec.end())
            {
                RCLCPP_INFO(this->get_logger(),"engineering is invalid.");
                return;
            }
                
        }
        {
            auto iter = std::find_if(_optionVec.begin(), _optionVec.end(), [&option](const std::string& optionVec)
            {
                return EndsWith(optionVec, option);
            });
            if(iter == _optionVec.end())
            {
                RCLCPP_INFO(this->get_logger(),"option is invalid.");
                return;
            }
        }
        std::unordered_map<std::string, int> umapEng = LUTEngineering();
        std::unordered_map<std::string, int>::const_iterator iterEng = umapEng.find(engineering);
        if(iterEng == umapEng.end())
            return;
        const int ENGINEER = iterEng->second;

        std::unordered_map<std::string, int> umapOp = LUTOption();
        std::unordered_map<std::string, int>::const_iterator iterOp = umapOp.find(option);
        if(iterOp == umapOp.end())
            return;
        const int OBJECT = iterOp->second;

        std::cout << "iterEng -> first: " << iterEng -> first << std::endl;
        std::cout << "iterEng -> second: " << iterEng -> second << std::endl;
        std::cout << "iterOp -> first: " << iterOp -> first << std::endl;
        std::cout << "iterOp -> second: " << iterOp -> second << std::endl;

        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
        unsigned int width = pointCloudCPtr->width;
        unsigned int height = pointCloudCPtr->height;
        unsigned int pointCloudSize = width * height;

        pointCloudPtr->points.resize(pointCloudSize);

        for(int i = 0; i < pointCloudSize; i++)
        {
            pcl::PointXYZI point;
            point.x = (pointCloudCPtr->x)[i];
            point.y = (pointCloudCPtr->y)[i];
            point.z = (pointCloudCPtr->z)[i];
            point.intensity = (pointCloudCPtr->intensity)[i];
            pointCloudPtr->points[i] = point;
        }

        OpeningData openingDate;

        openingDate.id = pointCloudCPtr->id;
        openingDate.engineering = pointCloudCPtr->engineering.c_str();
        openingDate.option = pointCloudCPtr->option.c_str();
        openingDate.angle = pointCloudCPtr->angle;
        openingDate.frameID = pointCloudCPtr->frame_id;
        openingDate.pointCloudPtr = std::move(pointCloudPtr);

        if(openingDate.frameID > 0)
        {
            _openingDataVec.push_back(openingDate);
            std::vector<float> result(2, -1);
            float code = 100;
            PubResult(code, result);
            return;
        }
        else
        {
            float code = 0;
            PubResult(code, Calculate(ENGINEER, OBJECT));
        }    

        return; 
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "Exception in SubPointCloud(): %s", e.what());
        rclcpp::shutdown();
    }
    catch(...)
    {
        RCLCPP_INFO(this->get_logger(),"Exception in SubPointCloud(): unknown");
        rclcpp::shutdown();
    }
}


void OpeningSize::SubInclinometer(shared_interfaces::msg::Inclinometer::UniquePtr ptr) 
{
    if(_status < 0)
        return;

    float angleX = ptr->angle_x;
    float angleY = ptr->angle_y;

    _inclinometer.SetAngleXY(angleX, angleY);
}


void OpeningSize::PubResult(const float& code, const std::vector<float>& result)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "initialize and publish a result.");
        return;
    }
    auto msg = std::make_unique<shared_interfaces::msg::MeasurementResult>();
    msg -> name = "";
    msg -> code = code;
    msg -> data = result;
    _pubResult->publish(std::move(msg));

    RCLCPP_INFO(this->get_logger(), "publish result: %f, %f.", result[0], result[1]);
}

void OpeningSize::PubStatus(const std::string& status)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "initialize and publish a status.");
        return;
    }

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = status;
    _pubStatus->publish(std::move(msg));

    RCLCPP_INFO(this->get_logger(), "publish status: %s.", status.c_str());
}

std::vector<float> OpeningSize::Calculate(
    const int& engineering,
    const int& object)
{
    std::cout << "Calculate" << std::endl;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> pointCloudVec;
    pcl::PointCloud<pcl::PointXYZI> pointCloudOutput;
    if(Convert(pointCloudVec) < 0)
        return {-1,-1};
    FSMeasureImpl sfm;

    std::vector<float> result;
    float width = -1;
    float height = -1;

    switch (object)
    {
    case DOOR:
        sfm.RunDoorMeasure(pointCloudVec, pointCloudOutput, width, height);
        result.push_back(width);
        result.push_back(height);
        break;
    case WINDOW:
        sfm.RunWindowMeasure(pointCloudVec, pointCloudOutput, width, height);
        result.push_back(width);
        result.push_back(height);
        break;
    default:
        break;
    }
    return result;
}

int OpeningSize::Convert(
    std::vector<pcl::PointCloud<pcl::PointXYZI>>& pointCloudVec)
{
    if(pointCloudVec.size() != 4)
        return -1;
    
    pointCloudVec.reserve(_openingDataVec.size());
    for(auto& data : _openingDataVec)
        pointCloudVec.push_back(*(data.pointCloudPtr));

    return 0;
}

}//am

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(am::OpeningSize)