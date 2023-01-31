#include "architecture_measurement/masonry_surface_flatness.hpp"

namespace am
{

MasonrySurfaceFlatness::MasonrySurfaceFlatness() {}

MasonrySurfaceFlatness::~MasonrySurfaceFlatness() {}

std::vector<float> MasonrySurfaceFlatness::Calculate(
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtr, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudFilterPtr,
    pcl::PointIndices::Ptr indicesFilterPtr, 
    cv::Mat& mortarJointImage,
    const int& engineering,
    const int& object, 
    const int& method)
{
// #ifdef AM_TEST
//     std::cout << "pointCloudPtr->points size: " << pointCloudPtr->points.size() << std::endl;
//     std::cout << "pointCloudFilterPtr->points size: " << pointCloudFilterPtr->points.size() << std::endl;
//     std::cout << "indicesFilterPtr->indices size: " << indicesFilterPtr->indices.size() << std::endl;
// #endif //AM_TEST

    if(pointCloudPtr->points.empty())
    {
        std::string error = "Error in Calculate(): PointCloud is empty.";
        throw std::runtime_error(error);
    }
    if(pointCloudFilterPtr->points.empty())
    {
        std::string error = "Error in Calculate(): PointCloudFilter is empty.";
        throw std::runtime_error(error);
    }
    if(indicesFilterPtr->indices.empty())
    {
        std::string error = "Error in Calculate(): indicesFilter is empty.";
        throw std::runtime_error(error);
    }

    //倾角仪坐标系和世界水平坐标系之间的变换矩阵         
    Eigen::Matrix3f W_R_I;
    float angleX_tmp, angleY_tmp;
    _inclinometer.GetAngleXY(angleX_tmp, angleY_tmp);
    _inclinometer.GetMatrix(W_R_I);
    Eigen::Matrix3f W_R_C = W_R_I * _I_R_C;

    CloudPretreatmentEx cp;

    //coordinate transformation
    Eigen::Vector3f transform{0.,0.,0.};
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrT(new pcl::PointCloud<pcl::PointXYZI>());
    cp.PointCloudTransformation(pointCloudFilterPtr, pointCloudPtrT, W_R_C, transform);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrOriginT(new pcl::PointCloud<pcl::PointXYZI>);
    cp.PointCloudTransformation(pointCloudPtr, pointCloudPtrOriginT, W_R_C, transform);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointCloudPtrVec;
    std::vector<pcl::PointIndices::Ptr> pointCloudIndicesVec;
    Pretreatment(pointCloudPtrT, pointCloudPtrVec, pointCloudIndicesVec, W_R_C, engineering, object);

    //screen point cloud
    int index = -1;
    SortCloud(pointCloudPtrVec, index, engineering, object);

    //extract optimal point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudOptimalPtr(pointCloudPtrVec[index]); //optimal point cloud
    pcl::PointIndices::Ptr& indicesOptimalMapPtr(pointCloudIndicesVec[index]); //optimal point cloud indices mapping

    pcl::PointIndices::Ptr indicesOptimalPtr(new pcl::PointIndices); //optimal point cloud indices
    indicesOptimalPtr -> indices.resize(indicesOptimalMapPtr -> indices.size());

    for(std::size_t i = 0; i < indicesOptimalMapPtr -> indices.size(); i++)
    {   
        auto& iom = indicesOptimalMapPtr->indices[i];
        auto& io = indicesOptimalPtr->indices[i];

        io = indicesFilterPtr->indices[iom];
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPretreatedPtr(new pcl::PointCloud<pcl::PointXYZI>);
    if(method == IMAGE)
    {
        // std::cout << "-------- method is image" << std::endl;
        cv::Mat maskImage(_imageHeight, _imageWidth, CV_8UC1, cv::Scalar::all(0));
        _masonry.CreateMaskMap(indicesOptimalPtr, maskImage);

        pcl::PointIndices::Ptr mortarJointIndicesPtr(new pcl::PointIndices);
        _masonry.ExtractMortarJoint(mortarJointImage, maskImage, mortarJointIndicesPtr);
        
        cp.Extract(pointCloudPtrOriginT, mortarJointIndicesPtr, pointCloudPretreatedPtr, true, false);

// #ifdef AM_TEST
        // FileIO fo;
        // std::string fileName = "/home/ubuntu/tmp/cloud_pretreated.txt";
        // fo.SetProperty(fileName);
        // fo.Write(pointCloudPretreatedPtr);

        // fo.SetProperty("/home/ubuntu/tmp/pointCloudPtrOriginT.txt");
        // fo.Write(pointCloudPtrOriginT);
// #endif //AM_TEST        
    }

    std::vector<float> result = VirtualRuler(pointCloudPretreatedPtr, W_R_C);
    return result;
}

int MasonrySurfaceFlatness::Pretreatment(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr,
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& pointCloudPtrVec,
    std::vector<pcl::PointIndices::Ptr>& pointIndicesPtrVec,
    const Eigen::Matrix3f& W_R_C,
    const int& engineering,
    const int& object)
{
    CloudPretreatmentEx cp;

    //claculate normal vector
    pcl::PointCloud<pcl::Normal>::Ptr pointCloudNormalPtr(new pcl::PointCloud<pcl::Normal>);
    cp.NormalEstimation(pointCloudPtr, pointCloudNormalPtr, MASONRY);

    //plane segmentation
    cp.PlaneSegmentation(pointCloudPtr, pointCloudNormalPtr, pointCloudPtrVec, pointIndicesPtrVec, MASONRY);

    if(pointCloudPtrVec.empty() || pointIndicesPtrVec.empty())
    {
        std::string error = "Error in Calculate(): pointCloudPtrVec is empty || pointIndicesPtrVec is empty.";    
        throw std::runtime_error(error);
    }
// #ifdef AM_TEST    
//     std::cout << "pointCloudPtrVec size: " << pointCloudPtrVec.size() << std::endl;
//     for(std::size_t i = 0; i < pointCloudPtrVec.size(); i++)
//     {
//         std::cout << "pointCloudPtrVec[" << i << "] size: " << pointCloudPtrVec[i]->points.size() << std::endl;
//     }
// #endif //AM_TEST

    return 0;
}

int MasonrySurfaceFlatness::SetImageWidth(const float& imageWidth)
{
    _imageWidth = imageWidth;
    _masonry.SetImageWidth(imageWidth);

    return 0;
}

int MasonrySurfaceFlatness::SetImageHeight(const float& imageHeight)
{
    _imageHeight = imageHeight;
    _masonry.SetImageHeight(imageHeight);

    return 0;
}


int MasonrySurfaceFlatness::SortCloud(
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& pointCloudPtrVec,
    int& index,
    const int& engineering,
    const int& object)
{
    CloudHeaderVector cds(pointCloudPtrVec.size());
    for(std::size_t i = 0; i < pointCloudPtrVec.size(); i++)
    {        
        CloudHeader cd{pointCloudPtrVec[i], i};
        cds[i] = std::move(cd);
    }

    //remove ground and roof point cloud
    if(object == SIDEWALL)
    {
        auto cds_end = std::remove_if(cds.begin(),cds.end(),
            [](CloudHeader cd)
            {
                bool boolean = std::fabs(cd.GetNormalVector().dot(Eigen::Vector3f::UnitZ())) > THRESHOLD;
// #ifdef AM_TEST
//                 std::cout << "cd.GetNormalVector(): " << cd.GetNormalVector().transpose() << std::endl;
//                 std::cout << "dot_value > THRESHOLD: " << boolean << std::endl;
// #endif //AM_TEST                
                return boolean;
            });
        cds.erase(cds_end, cds.end());
        if(cds.empty())
        {                    
            std::string error = "Error in SortCloud():  cds (remove horizontal plane) is empty.";      
            throw std::runtime_error(error);   
        }
    }

    //remove sidewall point cloud
    else
    {
        auto cds_end = std::remove_if(cds.begin(),cds.end(),
            [](CloudHeader cd)
            {
                bool boolean = std::fabs(cd.GetNormalVector().dot(Eigen::Vector3f::UnitZ())) < THRESHOLD;
// #ifdef AM_TEST
//                 std::cout << "cd.GetNormalVector(): " << cd.GetNormalVector().transpose() << std::endl;
//                 std::cout << "dot_value < THRESHOLD: " << boolean << std::endl;
// #endif //AM_TEST                
                return boolean;
            });
        cds.erase(cds_end, cds.end());
        if(cds.empty())
        {
            std::string error = "Error in SortCloud():  cds (remove vertical plane) is empty";
            throw std::runtime_error(error);   
        }

        //remove roof point cloud
        if(object == GROUND)
        {
            cds_end = std::remove_if(cds.begin(),cds.end(),
                [](CloudHeader cd)
                {
// #ifdef AM_TEST
//                     std::cout << "cd.GetMassCenter()(2): " << cd.GetMassCenter()(2) << std::endl;
// #endif //AM_TEST
                    return cd.GetMassCenter()(2) > 0;
                });
            cds.erase(cds_end, cds.end());
            if(cds.empty())
            {
                std::string error = "Error in SortCloud():  cds size (remove roof plane) < 1.";
                throw std::runtime_error(error);   
            }
        }

        //remove ground point cloud
        else if(object == ROOF)
        {
            cds_end = std::remove_if(cds.begin(),cds.end(),
                [](CloudHeader cd)
                {
// #ifdef AM_TEST
//                     std::cout << "cd.GetMassCenter()(2): " << cd.GetMassCenter()(2) << std::endl;
// #endif //AM_TEST
                    return cd.GetMassCenter()(2) < 0;
                });//去除地面
            cds.erase(cds_end, cds.end());
            if(cds.empty())
            {
                std::string error = "Error in SortCloud():  cds size (remove ground plane) < 1.";
                throw std::runtime_error(error);   
            }
        }
        else
        {
            std::string error = "Error in SortCloud(): object is invalid.";
            throw std::runtime_error(error);
        }
    }

    std::sort(cds.begin(), cds.end(), SortSize);
    index = cds[0].GetIndex();
    if(index < 0)
    {
        std::string error = "Error in Calculate(): index < 0.";
        throw std::runtime_error(error);
    }

    return 0;
}

std::vector<float> MasonrySurfaceFlatness::VirtualRuler(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr, 
    const Eigen::Matrix3f& W_R_C)
{
    // std::cout << "-------- start virtual ruler" << std::endl;
    //点云处理对象
    CloudPretreatmentEx cp;

    Eigen::Vector3f horizontalPlaneInCamera{0.,1.,0.};//相机坐标系Y轴矢量
    Eigen::Vector3f verticalPlaneInCamera{1.,0.,0.};//相机坐标系X轴矢量
    Eigen::Vector3f leftFallingPlaneInCamera{std::sqrt(0.5), std::sqrt(0.5), 0.};//相机坐标系X轴和Y轴角平分线向量
    Eigen::Vector3f rightFallingPlaneInCamera{std::sqrt(0.5), -std::sqrt(0.5), 0.};//相机坐标系X轴和-Y轴角平分线向量

    Eigen::Vector3f horizontalPlaneInWorld = W_R_C * horizontalPlaneInCamera;//相机坐标系Y轴矢在世界水平坐标系下的矢量
    Eigen::Vector3f verticalPlaneInWorld = W_R_C * verticalPlaneInCamera;//相机坐标系X轴矢量在世界水平坐标系下的矢量
    Eigen::Vector3f leftFallingPlaneInWorld = W_R_C * leftFallingPlaneInCamera;//相机坐标系X轴和Y轴角平分线向量在世界水平坐标系下的矢量
    Eigen::Vector3f rightFallingPlaneInWorld = W_R_C * rightFallingPlaneInCamera;//相机坐标系X轴和-Y轴角平分线向量在世界水平坐标系下的矢量

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrRuler0(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云--横
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrRuler1(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云--竖
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrRuler2(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云--撇
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrRuler3(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云--捺
    pcl::PointIndices::Ptr indicesPtrRuler0(new pcl::PointIndices);//靠尺点云索引--横
    pcl::PointIndices::Ptr indicesPtrRuler1(new pcl::PointIndices);//靠尺点云索引--竖
    pcl::PointIndices::Ptr indicesPtrRuler2(new pcl::PointIndices);//靠尺点云索引--撇
    pcl::PointIndices::Ptr indicesPtrRuler3(new pcl::PointIndices);//靠尺点云索引--捺

    FileIO of1;
    of1.SetProperty("/home/ubuntu/tmp/pointCloudPtr.txt");
    of1.Write(pointCloudPtr);

    //提取靠尺点云
    for (std::size_t i = 0; i < pointCloudPtr->points.size(); ++i) 
    {
        //三维点转化为向量
        const auto & p = pointCloudPtr->at(i);
        Eigen::Vector3f point{p.x, p.y, p.z};

        if(point(0) == 0 && point(1) == 0 && point(2) == 0)
            continue;

        float distance = -1;//点到平面的距离

        //筛选靠尺点云--横
        distance = std::fabs(point.dot(horizontalPlaneInWorld));
        if(distance < 12.5)//todo
        indicesPtrRuler0->indices.push_back(i);

        //筛选靠尺点云--竖
        distance = std::fabs(point.dot(verticalPlaneInWorld));
        if(distance < 12.5)//todo
        indicesPtrRuler1->indices.push_back(i);

        //筛选靠尺点云--撇
        distance = std::fabs(point.dot(leftFallingPlaneInWorld));
        if(distance < 12.5)//todo
        indicesPtrRuler2->indices.push_back(i);

        //筛选靠尺点云--捺
        distance = std::fabs(point.dot(rightFallingPlaneInWorld));
        if(distance < 12.5)//todo
        indicesPtrRuler3->indices.push_back(i);
    }

    float s0, s1, s2, s3;
    //提取靠尺点云--横，并计算平整度
    if(indicesPtrRuler0->indices.empty())
        s0 = -1;
    else
    {
        cp.Extract(pointCloudPtr, indicesPtrRuler0, pointCloudPtrRuler0, false, false);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudInliersPtr(new pcl::PointCloud<pcl::PointXYZI>);
        s0 = Statistic(pointCloudPtrRuler0, pointCloudInliersPtr);
        of1.SetProperty("/home/ubuntu/tmp/ruler_inliers0.txt");
        of1.Write(pointCloudInliersPtr);
    }

    //提取靠尺点云--竖，并计算平整度
    if(indicesPtrRuler1->indices.empty())
        s1 = -1;
    else
    {
        cp.Extract(pointCloudPtr, indicesPtrRuler1, pointCloudPtrRuler1, false, false);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudInliersPtr(new pcl::PointCloud<pcl::PointXYZI>);
        s1 = Statistic(pointCloudPtrRuler1, pointCloudInliersPtr);
                of1.SetProperty("/home/ubuntu/tmp/ruler_inliers1.txt");
        of1.Write(pointCloudInliersPtr);
    }

    //提取靠尺点云--撇，并计算平整度
    if(indicesPtrRuler2->indices.empty())
        s2 = -1;
    else
    {
        cp.Extract(pointCloudPtr, indicesPtrRuler2, pointCloudPtrRuler2, false, false);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudInliersPtr(new pcl::PointCloud<pcl::PointXYZI>);
        s2 = Statistic(pointCloudPtrRuler2, pointCloudInliersPtr);
                of1.SetProperty("/home/ubuntu/tmp/ruler_inliers2.txt");
        of1.Write(pointCloudInliersPtr);
    }

    //提取靠尺点云--捺，并计算平整度
    if(indicesPtrRuler3->indices.empty())
        s3 = -1;
    else
    {
        cp.Extract(pointCloudPtr, indicesPtrRuler3, pointCloudPtrRuler3, false, false);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudInliersPtr(new pcl::PointCloud<pcl::PointXYZI>);
        s3 = Statistic(pointCloudPtrRuler3, pointCloudInliersPtr);
                of1.SetProperty("/home/ubuntu/tmp/ruler_inliers3.txt");
        of1.Write(pointCloudInliersPtr);
    }

    FileIO of;
    of.SetProperty("/home/ubuntu/tmp/ruler0.txt");
    of.Write(pointCloudPtrRuler0);
    of.SetProperty("/home/ubuntu/tmp/ruler1.txt");
    of.Write(pointCloudPtrRuler1);
    of.SetProperty("/home/ubuntu/tmp/ruler2.txt");
    of.Write(pointCloudPtrRuler2);
    of.SetProperty("/home/ubuntu/tmp/ruler3.txt");
    of.Write(pointCloudPtrRuler3);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrTest(new pcl::PointCloud<pcl::PointXYZI>);
    // of.SetProperty("/home/ubuntu/tmp/ruler1.txt");
    // of.Read(pointCloudPtrTest);
    // float s_test = Statistic(pointCloudPtrTest);
    // std::cout << "s_test: " << s_test << std::endl;


    //四条靠尺点云合并
    // pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrRulers(new pcl::PointCloud<pcl::PointXYZI>);
    // *pointCloudPtrRulers += *pointCloudPtrRuler0;
    // *pointCloudPtrRulers += *pointCloudPtrRuler1;
    // *pointCloudPtrRulers += *pointCloudPtrRuler2;
    // *pointCloudPtrRulers += *pointCloudPtrRuler3;

    // _pGUI.SetPointCloudGround(pointCloudPtr);
    // _pGUI.SetPointCloudRulers(pointCloudPtrRulers);
    // _pGUI.SavePointCloud();

    // std::cout << "++++++++ test4" << std::endl;
    std::cout << "s0: " << s0 << std::endl;
    std::cout << "s1: " << s1 << std::endl;
    std::cout << "s2: " << s2 << std::endl;
    std::cout << "s3: " << s3 << std::endl;
    return {s0, s1, s2, s3};
}


float MasonrySurfaceFlatness::Statistic(pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr, pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudInliersPtr)
{
    //判断点云数据是否为空
    if(pointCloudPtr->points.empty())
    {
        std::string error = "Error in Statistic(): cloud is empty.";
        throw std::runtime_error(error);
    }                

    //点云预处理对象
    CloudPretreatmentEx cp;

    //拟合平面

    pcl::PointIndices::Ptr pointIndicesPtr(new pcl::PointIndices);
    pcl::ModelCoefficients planeCoeffs = cp.Plane(pointCloudPtr,pointIndicesPtr);
    cp.Extract(pointCloudPtr, pointIndicesPtr, pointCloudInliersPtr, false, false);
    // std::cout << "plane coeffs: " << planeCoeffs.values[0] << " " << planeCoeffs.values[1] << " " << planeCoeffs.values[2] << " " << planeCoeffs.values[3] << std::endl;

    //计算点到平面的有向距离
    std::vector<float> distanceSigned;
    distanceSigned.reserve(pointCloudPtr->points.size());
    for (auto & p : *pointCloudPtr) 
    {
        float distance = pcl::pointToPlaneDistanceSigned<pcl::PointXYZI>(
            p,
            planeCoeffs.values[0],
            planeCoeffs.values[1],
            planeCoeffs.values[2],
            planeCoeffs.values[3]);

        distanceSigned.push_back(distance);
    }

    //计算点到平面的距离极差
    auto result = std::minmax_element (distanceSigned.begin(),distanceSigned.end());
    float distance = *result.second - *result.first;

    return distance;
}

int MasonrySurfaceFlatness::SetI_R_C(const Eigen::Matrix3f& I_R_C)
{
    _I_R_C = I_R_C;

    return 0;
}

int MasonrySurfaceFlatness::SetAngleXY(const float& angleX, const float& angleY)
{
    _inclinometer.SetAngleXY(angleX, angleY);

    return 0;
}

int MasonrySurfaceFlatness::SetFileName(const std::string& fileName)
{
    _pGUI.SetFileName(fileName);

    return 0;
}

}//namespace am

