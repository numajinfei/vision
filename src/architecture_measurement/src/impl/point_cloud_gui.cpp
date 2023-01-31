#include "architecture_measurement/impl/point_cloud_gui.hpp"

namespace am
{

PointCloudGUI::PointCloudGUI() {}

PointCloudGUI::~PointCloudGUI()
{
    _pointCloudPtr.reset();
}

int PointCloudGUI::SavePointCloud()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrT(new pcl::PointCloud<pcl::PointXYZI>);
    Transformation(pointCloudPtrT);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtrDR(new pcl::PointCloud<pcl::PointXYZI>);
    DuplicateRemoval(pointCloudPtrDR);

    FileIO of;
    of.SetProperty(_fileName);
    of.Write(pointCloudPtrDR);

    return 0;
}

int PointCloudGUI::SetPointCloudGround(pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr)
{
    *_pointCloudPtr += *pointCloudPtr;

    return 0;
}

int PointCloudGUI::SetPointCloudRulers(pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr)
{
    *_pointCloudPtr += *pointCloudPtr;

    return 0;
}

int PointCloudGUI::SetFileName(const std::string& fileName)
{
    _fileName = fileName;

    return 0;
}

int PointCloudGUI::DuplicateRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr)
{

    CloudPretreatment cp;
    cp.DuplicateRemoval(_pointCloudPtr, pointCloudPtr);

    return 0;
}

int PointCloudGUI::Transformation(pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudPtr)
{
    CloudPretreatment cp;
    pcl::PointXYZI minPoint, maxPoint;
    cp.GetMinMax3D(_pointCloudPtr, minPoint, maxPoint);

    float offsetX = (minPoint.x + maxPoint.x) / 2;
    float offsetY = (minPoint.y + maxPoint.y) / 2;
    float offsetZ = (minPoint.z + maxPoint.z) / 2;

    Eigen::Vector3f translation{-offsetX, -offsetY , -offsetZ};
    Eigen::Matrix3f rotation;
    rotation = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
    
    cp.PointCloudTransformation(_pointCloudPtr, pointCloudPtr, rotation, translation);

    return 0;
}

}//namespace am