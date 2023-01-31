#include "architecture_measurement/masonry_perpendicularity.hpp"

namespace am
{

MasonryPerpendicularity::MasonryPerpendicularity() {}

MasonryPerpendicularity::~MasonryPerpendicularity() {}

std::vector<float> MasonryPerpendicularity::Calculate(
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtr, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudFilterPtr,
    pcl::PointIndices::Ptr indicesFilterPtr, 
    cv::Mat& mortarJointImage,
    const int& engineering,
    const int& object, 
    const int& method)
{
#ifdef AM_TEST
    std::cout << "pointCloudPtr->points size: " << pointCloudPtr->points.size() << std::endl;
    std::cout << "pointCloudFilterPtr->points size: " << pointCloudFilterPtr->points.size() << std::endl;
    std::cout << "indicesFilterPtr->indices size: " << indicesFilterPtr->indices.size() << std::endl;
#endif //AM_TEST

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

    //transform matrix
    Eigen::Matrix3f W_R_I;
    _inclinometer.GetMatrix(W_R_I);
    Eigen::Matrix3f W_R_C = W_R_I * _I_R_C;

    CloudPretreatmentEx cp;

    //coordinate transformation
    Eigen::Vector3f transform{0.,0.,0.};
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrT(new pcl::PointCloud<pcl::PointXYZI>());
    cp.PointCloudTransformation(pointCloudFilterPtr, pointCloudPtrT, W_R_C, transform);

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
#ifdef AM_TEST
    std::cout << "indicesOptimalPtr -> indices size: " << indicesOptimalPtr -> indices.size() << std::endl;
#endif //AM_TEST

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrPretreated(new pcl::PointCloud<pcl::PointXYZI>);
    if(method == IMAGE)
    {
        cv::Mat maskImage(_imageHeight, _imageWidth, CV_8UC1, cv::Scalar::all(0));
        _masonry.CreateMaskMap(indicesOptimalPtr, maskImage);

        pcl::PointIndices::Ptr mortarJointIndicesPtr(new pcl::PointIndices);
        _masonry.ExtractMortarJoint(mortarJointImage, maskImage, mortarJointIndicesPtr);
        
        cp.Extract(pointCloudPtr, mortarJointIndicesPtr, pointCloudPtrPretreated, true);

// #ifdef AM_TEST
//         FileIO fo;
//         std::string fileName = "../result/cloud_pretreated.txt";
//         fo.SetProperty(fileName);
//         fo.Write(pointCloudPtrPretreated);
// #endif //AM_TEST        
    }

    std::vector<float> result;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrRulers(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云

    switch (object)
    {
        case SIDEWALL:  
        {
            result = VirtualRulerSideWall(pointCloudPtrPretreated, pointCloudPtrRulers, W_R_C, engineering, object);
            break;
        }
        case PILLAR:
        {
            result = VirtualRulerPillar(pointCloudPtrPretreated, pointCloudPtrRulers, W_R_C, engineering, object);
            break;
        }
        default:
        {
            std::string error = "Error in Calculate(): type is invalid.";
            throw std::runtime_error(error);  
        }                    
    }
    
    _pGUI.SetPointCloudGround(pointCloudPtr);
    _pGUI.SetPointCloudRulers(pointCloudPtrRulers);
    _pGUI.SavePointCloud();

    return {result};
}

int MasonryPerpendicularity::Pretreatment(
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
#ifdef AM_TEST    
    std::cout << "pointCloudPtrVec size: " << pointCloudPtrVec.size() << std::endl;
    for(std::size_t i = 0; i < pointCloudPtrVec.size(); i++)
    {
        std::cout << "pointCloudPtrVec[" << i << "] size: " << pointCloudPtrVec[i]->points.size() << std::endl;
    }
#endif //AM_TEST

    return 0;
}

int MasonryPerpendicularity::SetImageWidth(const float& imageWidth)
{
    _imageWidth = imageWidth;

    return 0;
}

int MasonryPerpendicularity::SetImageHeight(const float& imageHeight)
{
    _imageHeight = imageHeight;

    return 0;
}

int MasonryPerpendicularity::SortCloud(
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& pointCloudPtrVec,
    int& index,
    const int& engineering,
    const int& object)
{   
    CloudHeaderVector cds(pointCloudPtrVec.size());

    for(int i = 0; i < pointCloudPtrVec.size(); i++)
    {        
        CloudHeader cd{pointCloudPtrVec[i], i};
        cds[i] = std::move(cd);
    }

#ifdef AM_TEST
    std::cout << "cds size (init): " << cds.size() << std::endl;
#endif //AM_TEST

    //祛除水平面
    auto cds_end = std::remove_if(cds.begin(),cds.end(),
        [](CloudHeader cd)
        {
            return std::fabs(cd.GetNormalVector().dot(Eigen::Vector3f::UnitZ())) > THRESHOLD;
        });
    cds.erase(cds_end, cds.end());
    if(cds.empty())
    {
        std::string error = "Error in SortCloud(): cds is empty.";
        throw std::runtime_error(error);
    }
#ifdef AM_TEST
    std::cout << "cds size (remove horizontal plane): " << cds.size() << std::endl;
#endif //AM_TEST
    switch(object)
    {
        case SIDEWALL://侧墙
        {                    
            std::sort(cds.begin(), cds.end(), SortSize);//按照点云的数量从大到小进行排序
            index = cds[0].GetIndex();//选择数量最多的点云平面
            break;
        }
        case PILLAR://柱
        {
            auto cds_end = std::remove_if(cds.begin(),cds.end(),
                [](CloudHeader cd)
                {
                    return std::fabs(cd.GetNormalVector().dot(Eigen::Vector3f::UnitX())) < THRESHOLD;
                });
            cds.erase(cds_end, cds.end());
            if(cds.empty())
            {
                std::string error = "Error in SortCloud(): cds is empty.";
                throw std::runtime_error(error);
            }
            std::sort(cds.begin(),cds.end(),SortMassCenterXL);//按照点云质心X坐标从小到大进行排序
            index = cds[0].GetIndex();//质心X坐标最小的点云平面
            break;
        }
        default:
        {                    
            std::string error = "Error in SortCloud(): type is invalid.";
            throw std::runtime_error(error);
        }
    }
    return 0;
}

std::vector<float> MasonryPerpendicularity::VirtualRulerSideWall(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtrOptimal,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtrRulers,
    const Eigen::Matrix3f& W_R_C,
    const int& engineering,
    const int& object)
{
    CloudPretreatmentEx cp;
    std::vector<float> result;

    pcl::ModelCoefficients planeCoeffs = cp.Plane(pointCloudPtrOptimal); //最优点云平面进行平面拟合，获取平面参数

    Eigen::Vector3f cameraAxisZ{0.,0.,1.}; //相机坐标系Z轴
    Eigen::Vector3f cameraAxisZInWorld = W_R_C * cameraAxisZ; //相机坐标系Z轴在水平坐标系下的矢量
    Eigen::Vector3f originPoint{0., 0., 0.}; //原点坐标

    Line3D<float> line{originPoint, cameraAxisZInWorld};//线对象
    Plane3D<float> plane{planeCoeffs.values[0],planeCoeffs.values[1],planeCoeffs.values[2],planeCoeffs.values[3]};//最优点云平面对象
    LineAndPlane<float> lineAndPlane{line, plane};//线和平面对象
    Point3D<float> intersectionPoint;//点对象
    lineAndPlane.GetIntersectionPoint(intersectionPoint);//计算点和平面之间的交点
    auto centerY = intersectionPoint.point(1);//求解交点y坐标

    for(int i = -2; i < 3; i++)
    {
        float binaryLeft = centerY + 2 * i * 50 - 25;//靠尺边界下边缘 //todo
        float binaryRight = centerY + 2 * i * 50 + 25;//靠尺边界上边缘
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrRuler(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云
        cp.ConditionRemoval(pointCloudPtrOptimal, pointCloudPtrRuler, am::AXIS_Y, binaryLeft, binaryRight);//筛选靠尺点云
        if(pointCloudPtrRuler -> size() < 100)
        {
            result.push_back(-1);
            continue;
        }

        *pointCloudPtrRulers += *pointCloudPtrRuler;

        result.push_back(Statistic(pointCloudPtrRuler));
    }

    return result;
}

std::vector<float> MasonryPerpendicularity::VirtualRulerPillar(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtrOptimal,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtrRulers,
    const Eigen::Matrix3f& W_R_C,
    const int& engineering,
    const int& object)
{
    CloudPretreatmentEx cp;
    std::vector<float> result;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrBoundary(new pcl::PointCloud<pcl::PointXYZI>);//点云平面边界
    pcl::PointCloud<pcl::Normal>::Ptr pointCloudPtrNormal(new pcl::PointCloud<pcl::Normal>);//点云法向量
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrLine(new pcl::PointCloud<pcl::PointXYZI>);//竖直方向上的直线
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrRemain(new pcl::PointCloud<pcl::PointXYZI>);//其余点

    cp.NormalEstimation(pointCloudPtrOptimal, pointCloudPtrNormal);//估计法向量
    cp.Boundary(pointCloudPtrOptimal,pointCloudPtrNormal,pointCloudPtrBoundary);//提取点云边界

    std::vector<VerticalBoundary> verticalBoundaryVec;//竖直边界
    //提取全部可能的竖直边界
    while(true)
    {
        pcl::PointIndices::Ptr indicesPtrLine(new pcl::PointIndices);//竖直方向上点的索引
        pcl::ModelCoefficients parallelLineCoeffs = cp.ParallelLine(pointCloudPtrBoundary,indicesPtrLine,Eigen::Vector3f::UnitZ(), 100);//拟合竖直边界，并计算参数
        cp.Extract(pointCloudPtrBoundary,indicesPtrLine,pointCloudPtrLine,false);//提取竖直边界
        cp.Extract(pointCloudPtrBoundary,indicesPtrLine,pointCloudPtrRemain,true);//提取其余点
        std::swap(pointCloudPtrBoundary->points, pointCloudPtrRemain->points);//将剩余点与边界点进行交换

        VerticalBoundary vb(parallelLineCoeffs, pointCloudPtrLine);//构造竖直边界
        verticalBoundaryVec.emplace_back(vb);
    }

    //将竖直边界按照边界的长短进行排序
    auto verticalBoundaryEnd = std::remove_if(verticalBoundaryVec.begin(), verticalBoundaryVec.end(), 
        [](VerticalBoundary& vb)
        {
            return vb._length < 500;//todo
        });
    verticalBoundaryVec.erase(verticalBoundaryEnd, verticalBoundaryVec.end());

    //判断竖直边界的数量
    if(verticalBoundaryVec.size() < 2)
    {
        std::string error = " Error in Calculate(), vertical boundary is invalid.";
        throw std::runtime_error(error);
    }

    std::sort(verticalBoundaryVec.begin(), verticalBoundaryVec.end(), SortVerticalBoundaryYL);//按照中心点Y坐标从小到大的顺序对竖直边界进行排序

    VerticalBoundary verticalBoundaryLeft = *verticalBoundaryVec.rbegin();//选择最左边的一条竖直边界构造竖直边缘对象
    VerticalBoundary verticalBoundaryRight = *verticalBoundaryVec.begin();//选择最右边的一条竖直边界构造竖直边缘对象

    float yl = verticalBoundaryLeft._y;//左侧竖直边界Y坐标
    float ys = verticalBoundaryRight._y;//右侧竖直边界Y坐标
    //计算竖直边界在Z方向上的相交范围
    if(yl - ys < 100)
    {
        std::string error = " Error in Calculate(), pillar section size < 100mm.";
        throw std::runtime_error(error);
    }

    float ym = (ys + yl)/2.0;

#ifdef AM_TEST
    std::cout << "ys: " << ys << std::endl;
    std::cout << "yl: " << yl << std::endl;
    std::cout << "ym: " << ym << std::endl;
#endif //AM_TEST

    //计算两个竖直边界与中心水平面之间的交点，及交点之间的方向向量
    Plane3D<float> planeXOY{0., 0., 1., 0.};

    auto lineCoeffsLeft = verticalBoundaryLeft._coeffs.values;//左竖直边界参数
    auto lineCoeffsRight = verticalBoundaryRight._coeffs.values;//右竖直边界参数

    Eigen::Vector3f linePointLeft{lineCoeffsLeft[0],lineCoeffsLeft[1],lineCoeffsLeft[2]};//左竖直边界点
    Eigen::Vector3f linePointRight{lineCoeffsRight[0],lineCoeffsRight[1],lineCoeffsRight[2]};//右竖直边界点
    Eigen::Vector3f lineVectorLeft{lineCoeffsLeft[3],lineCoeffsLeft[4],lineCoeffsLeft[5]};//左竖直边界方向向量
    Eigen::Vector3f lineVectorRight{lineCoeffsRight[3],lineCoeffsRight[4],lineCoeffsRight[5]};//右竖直边界方向向量

    Point3D<float> point3dLeft{linePointLeft};//左竖直边界点对象
    Point3D<float> point3dRight{linePointRight};//右竖直边界点对象
    Vector3D<float> vector3dLeft{lineVectorLeft};//左竖直边界方向向量对象
    Vector3D<float> vector3dRight{lineVectorRight};//右竖直边界方向向量对象
    Line3D<float> line3dLeft{point3dLeft, vector3dLeft};//左竖直边界线对象
    Line3D<float> line3dRight{point3dRight, vector3dRight};//右竖直边界线对象
    LineAndPlane<float> line3dLeftPlane{line3dLeft, planeXOY};//左竖直边界线和平面对象
    LineAndPlane<float> line3dRightPlane{line3dRight, planeXOY};//右竖直边界线和平面对象

    Point3D<float> point3dIntersectionLeft;//左竖直边界和水平面之间的交点对象
    Point3D<float> point3dIntersectionRight;//右竖直边界和水平面之间的交点对象
    line3dLeftPlane.GetIntersectionPoint(point3dIntersectionLeft);//求解交点
    line3dRightPlane.GetIntersectionPoint(point3dIntersectionRight);//求解交点

    Eigen::Vector3f directionVector = point3dIntersectionLeft.point - point3dIntersectionRight.point;//求解交点间方向向量
#ifdef AM_TEST
    std::cout << "directionVector: " << directionVector << std::endl;
#endif //AM_TEST
    directionVector /= directionVector.norm();//方向向量归一化
    if(directionVector[1] < 0)//将方向向量变换到与水平坐标系Y轴正方向同向
        directionVector = -directionVector;                    

    for(int i = -2; i < 3; i++)
    { 
        //提取靠尺点云
        float binaryRight = ym + 2 * i * 50 - directionVector(1) * 25;//靠尺点云右边界
        float binaryLeft = ym + 2 * i * 50 + directionVector(1) * 25;//靠尺点云左边界

        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrRuler(new pcl::PointCloud<pcl::PointXYZI>);//靠尺点云
        cp.ConditionRemoval(pointCloudPtrOptimal, pointCloudPtrRuler, am::AXIS_Y, binaryRight, binaryLeft);//提取靠尺点云
        if(pointCloudPtrRuler -> size() < 100)
        {
            result.push_back(-1);
            continue;
        }

        *pointCloudPtrRulers += *pointCloudPtrRuler;

        result.push_back(Statistic(pointCloudPtrRuler));
    }

    return result;
}

bool SortPointCloudZL(pcl::PointXYZI& point1, pcl::PointXYZI& point2)
{
    return point1.z < point2.z;
}


float MasonryPerpendicularity::Statistic(pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr)
{
    CloudPretreatmentEx cp;
    pcl::ModelCoefficients planeCoeffs = cp.Plane(pointCloudPtr);//靠尺点云平面参数
    Eigen::Vector3f NormalVector{planeCoeffs.values[0],planeCoeffs.values[1],planeCoeffs.values[2]};//靠尺点云平面法矢

    float angleRadian = std::fabs(std::acos(Eigen::Vector3f::UnitZ().dot(NormalVector)) - M_PI / 2.);//计算靠尺点云平面法矢与水平坐标系Z轴之间的夹角
    float angleDegree = std::fabs(Degree(angleRadian));//将夹角转换为角度，并求与90度之间的偏差

    std::sort(pointCloudPtr->points.begin(), pointCloudPtr->points.end(), SortPointCloudZL);
    float rulerDistance = std::fabs( (*pointCloudPtr->points.begin()).z - (*pointCloudPtr->points.rbegin()).z );

    float result = rulerDistance * std::tan(angleRadian) / _hyperparameter;

#ifdef AM_TEST
        std::cout << "angleDegree: " << angleDegree << std::endl;
        std::cout << "rulerDistance: " << rulerDistance  << "    _hyperparameter: " << _hyperparameter << std::endl;
        std::cout << "result: " << result << std::endl;
#endif //AM_TEST

    return result;
}

int MasonryPerpendicularity::SetI_R_C(const Eigen::Matrix3f& I_R_C)
{
    _I_R_C = I_R_C;

    return 0;
}

int MasonryPerpendicularity::SetAngleXY(const float& angleX, const float& angleY)
{
    _inclinometer.SetAngleXY(angleX, angleY);

    return 0;
}

int MasonryPerpendicularity::SetFileName(const std::string& fileName)
{
    _pGUI.SetFileName(fileName);

    return 0;
}

int MasonryPerpendicularity::SetHyperParameter(const float& hyperparameter)
{
    _hyperparameter = hyperparameter;

    return 0;
}

} //namespace am