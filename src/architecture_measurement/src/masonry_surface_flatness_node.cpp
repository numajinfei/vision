#include "architecture_measurement/masonry_surface_flatness_node.hpp"

namespace am
{

MasonrySurfaceFlatnessNode::MasonrySurfaceFlatnessNode(const rclcpp::NodeOptions& option) : rclcpp::Node("masonry_surface_flatness_node", option)
{
    _initThread = std::thread(&MasonrySurfaceFlatnessNode::Init, this);
}

MasonrySurfaceFlatnessNode::~MasonrySurfaceFlatnessNode()
{
    _initThread.join();

    _subPointCloud.reset();
    _subImage.reset();
    _subInclinometer.reset();
    _pubResult.reset();
    _pubStatus.reset();

    RCLCPP_INFO(this->get_logger(),"MasonrySurfaceFlatnessNode destroyed successfully.");
}


void MasonrySurfaceFlatnessNode::Init() 
{
    try
    {
        _status = -1;

        InitializeParameters();

        UpdateParameters();

        _subPointCloud = this->create_subscription<shared_interfaces::msg::PointCloudC>(
            _subPointCloudName, 
            5, 
            std::bind(&MasonrySurfaceFlatnessNode::SubPointCloud, this, std::placeholders::_1));

        _subImage = this->create_subscription<shared_interfaces::msg::ImageC>(
            _subImageName,
            5,
            std::bind(&MasonrySurfaceFlatnessNode::SubImage, this, std::placeholders::_1));

        _subInclinometer = this->create_subscription<shared_interfaces::msg::Inclinometer>(
            _subInclinometerName, 
            5, 
            std::bind(&MasonrySurfaceFlatnessNode::SubInclinometer, this, std::placeholders::_1));

        _pubResult = this->create_publisher<shared_interfaces::msg::MeasurementResult>(_pubResultName, 1);

        _pubStatus = this->create_publisher<std_msgs::msg::String>(_pubStatusName, 1);

        _status = 0;

        RCLCPP_INFO(this->get_logger(), "MasonrySurfaceFlatnessNode initialized successfully.");
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "Exception in MasonrySurfaceFlatnessNode initializer: %s", e.what());
        rclcpp::shutdown();
    }
    catch(...)
    {
        RCLCPP_INFO(this->get_logger(),"Exception in MasonrySurfaceFlatnessNode initializer: unknown");
        rclcpp::shutdown();
    }
}


void MasonrySurfaceFlatnessNode::InitializeParameters()
{
    this->declare_parameter<std::vector<std::string>>("engineering", {"Normal"});
    this->declare_parameter<std::vector<std::string>>("option",{"StoreyHeight"});
    this->declare_parameter<std::vector<double>>("I_R_C",{1., 0., 0., 0., 1., 0., 0., 0., 1.});
    this->declare_parameter<std::string>("fileName","../../result/pointCloud.txt");
}

void MasonrySurfaceFlatnessNode::UpdateParameters()
{
    std::vector<double> I_R_C_vector;  

    this->get_parameter("engineering", _engineeringVec);
    this->get_parameter("option", _optionVec); 
    this->get_parameter("I_R_C",I_R_C_vector);
    this->get_parameter("fileName",_fileName);

    double* I_R_C_vector_ptr = I_R_C_vector.data();
    Eigen::Matrix3d _I_R_C_tmp = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> >(I_R_C_vector_ptr);
    _I_R_C = _I_R_C_tmp.cast<float>();

    _masonrySurfaceFlatness.SetI_R_C(_I_R_C);
    _masonrySurfaceFlatness.SetFileName(_fileName);
}

void MasonrySurfaceFlatnessNode::SubPointCloud(shared_interfaces::msg::PointCloudC::UniquePtr pointCloudCPtr)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(),"initialize and subscribe a point cloud.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "sub a point cloud");
    const long id = pointCloudCPtr->id;
    const std::string engineering = pointCloudCPtr->engineering.c_str();
    const std::string option = pointCloudCPtr->option.c_str();
    const float angle = pointCloudCPtr->angle;
    const int frameID = pointCloudCPtr->frame_id;

    {
        auto iter = std::find_if(_engineeringVec.begin(), _engineeringVec.end(), [&engineering](const std::string& engineeringVec)
        {
            return EndsWith(engineeringVec, engineering);
        });
        if(iter == _engineeringVec.end())
            return;
    }
    {
        auto iter = std::find_if(_optionVec.begin(), _optionVec.end(), [&option](const std::string& optionVec)
        {
            return EndsWith(optionVec, option);
        });
        if(iter == _optionVec.end())
            return;
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
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudFilterPtr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointIndices::Ptr indicesFilterPtr(new pcl::PointIndices);

    unsigned int width = pointCloudCPtr->width;
    unsigned int height = pointCloudCPtr->height;
    unsigned int pointCloudSize = width * height;

    pointCloudPtr->points.resize(pointCloudSize);
    pointCloudFilterPtr->points.reserve(pointCloudSize);
    indicesFilterPtr->indices.reserve(pointCloudSize);

    int serial = 0;

    for(int i = 0; i < pointCloudSize; i++)
    {
        pcl::PointXYZI point;
        point.x = (pointCloudCPtr->x)[i];
        point.y = (pointCloudCPtr->y)[i];
        point.z = (pointCloudCPtr->z)[i];
        point.intensity = (pointCloudCPtr->intensity)[i];
        pointCloudPtr->points[i] = point;

        if(point.x == 0 && point.y == 0; point.z == 0)
        {
            ++serial;
            continue;
        }

        pointCloudFilterPtr->points.push_back(point);
        indicesFilterPtr->indices.push_back(serial++);
    }

    // _combinationDate._pointCloudPtr = pointCloudPtr;
    _combinationDate._pointCloudID = id;

    // int loop = 0;
    // while(true)
    // {
    //     ++loop;
    //     std::cout << "_combinationDate complete: " << _combinationDate.Complete() << std::endl;
    //     if(_combinationDate.Complete())
    //     {
    //         break;
    //     }
    //     // std::this_thread::sleep_for(100ms);
    //     else
    //     {
    //         std::this_thread::sleep_for(500ms);
    //         std::cout << "loop: " << loop << std::endl;
    //         if(loop > 50)
    //         {                
    //             std::vector<float> result(4, -1);
    //             PubResult(result);
    //             RCLCPP_INFO(this->get_logger(), "there is no image");                
    //             return;
    //         } 
    //     }            
    // }

    std::vector<float> result;
    result = _masonrySurfaceFlatness.Calculate(pointCloudPtr, pointCloudFilterPtr, indicesFilterPtr, 
            _combinationDate._image, ENGINEER, OBJECT, Method::IMAGE);
    PubResult(option, frameID, result);
}

void MasonrySurfaceFlatnessNode::SubImage(shared_interfaces::msg::ImageC::UniquePtr ptr)
{
    RCLCPP_INFO(this->get_logger(), "sub an image form mortar_joint node");
    if(ptr->engineering == "Masonry" && ptr->option == "SideWallFlatness")
    {
        // RCLCPP_INFO(this->get_logger(), "sub an image form mortar_joint node");
        _combinationDate._image = cv::Mat(ptr->image.height, ptr->image.width, CV_8UC1, ptr->image.data.data());
        _masonrySurfaceFlatness.SetImageWidth(ptr->image.width);
        _masonrySurfaceFlatness.SetImageHeight(ptr->image.height);
        _combinationDate._imageID = ptr->id;
    }

}

void MasonrySurfaceFlatnessNode::SubInclinometer(shared_interfaces::msg::Inclinometer::UniquePtr ptr)
{
    if(_status < 0)
        return;

    float angleX = ptr->angle_x;
    float angleY = ptr->angle_y;

    _masonrySurfaceFlatness.SetAngleXY(angleX, angleY);
}

void MasonrySurfaceFlatnessNode::PubResult(const std::string& name, const float& code, const std::vector<float>& result)
{
    if(_status < 0)
    {
        RCLCPP_INFO(this->get_logger(), "initialize and publish a result.");
        return;
    }
    auto msg = std::make_unique<shared_interfaces::msg::MeasurementResult>();
    msg -> name = name;
    msg -> code = code;
    msg -> data = result;
    _pubResult->publish(std::move(msg));

    RCLCPP_INFO(this->get_logger(), "publish result: %f, %f, %f, %f.", result[0], result[1], result[2], result[3]);
}

void MasonrySurfaceFlatnessNode::PubStatus(const std::string& status)
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

}//am

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(am::MasonrySurfaceFlatnessNode)