#include "architecture_measurement/masonry.hpp"

#include <fstream>

namespace am
{

Masonry::Masonry() {}

Masonry::~Masonry() {}

int Masonry::Search(const cv::Point& point, int& index, const int& method)
{
    switch(method)
    {
        case 0:
        {
            if(point.x == 0)
                index = point.y;
            index = (point.x) * _imageHeight + point.y;
            break;
        }
        case 1:
        {
            if(point.y == 0)
                index = point.x;
            index = (point.y) * _imageWidth + point.x;
            break;
        }
        default:
        {
            std::string error = "Error in Search(): wrong method.";
            int errorCode = -1;
            throw std::runtime_error(error);
        }
    }
    return 0;
}

int Masonry::Search(const int& index, cv::Point& point, const int method)
{
    // std::cout << "search" << std::endl;
    switch(method)
    {
        case 0:
        {
            point.x = index / _imageHeight;
            point.y = index % _imageHeight;
            // std::cout << "point: " << point.x << " " << point.y << std::endl;
            break;
        }
        case 1:
        {
            point.y = index / _imageWidth;
            point.x = index % _imageWidth;

            // std::cout << "point: " << point.x << " " << point.y << std::endl;
            break;
        }
        default:
        {
            std::string error = "Error in Search(): wrong method.";
            throw std::runtime_error(error);
        }
    }
    return 0;
}


int Masonry::CreateMaskMap(pcl::PointIndices::Ptr& indices, cv::Mat& maskMap)
{
    std::cout << "-------- start CreateMaskMap" << std::endl;
    cv::Mat tmp(maskMap.rows, maskMap.cols, CV_8UC1, cv::Scalar::all(0));
    std::size_t indicesSize = indices->indices.size();
    // std::cout << "-------- indicesSize: " << indicesSize << std::endl;

    // std::cout << "_imageWidth: " << _imageWidth <<std::endl;
    // std::cout << "_imageHeight: " << _imageHeight << std::endl;
    for(std::size_t i = 0; i < indicesSize; ++i)
    {
        auto index = indices->indices[i];
        // std::cout << "index: " << index << std::endl;

        cv::Point pixelCoordinate;
        Search(index, pixelCoordinate, ROW_MAJOR);

        tmp.at<uchar>(pixelCoordinate.y, pixelCoordinate.x) = 255;
    }
    MedianFilter(tmp, maskMap);
    std::cout << "--------after CreateMaskMap" << std::endl;
    return 0;
}

int Masonry::ExtractMortarJoint(cv::Mat& mortarJointImage, cv::Mat& maskImage, pcl::PointIndices::Ptr& indicesPtr)
{
    std::cout << "-------- start extract mortar joint" << std::endl;

    if(mortarJointImage.depth() != maskImage.depth())
    {
        std::string error = "Error in ExtractMortarJoint(): mortarJointImage.depth() != maskImage.depth().";
        throw std::runtime_error(error);
    }

    if(mortarJointImage.size != maskImage.size)
    {
        std::string error = "Error in ExtractMortarJoint(): mortarJointImage.size != maskImage.size.";
        throw std::runtime_error(error);
    }
    std::size_t height = mortarJointImage.rows;
    std::size_t width = mortarJointImage.cols;

    indicesPtr -> indices.reserve(height * width);
    cv::Mat image_and(_imageHeight, _imageWidth, CV_8UC1, cv::Scalar::all(0));
    if(mortarJointImage.depth() != image_and.depth())
    {
        std::string error = "Error in ExtractMortarJoint(): mortarJointImage.depth() != maskImage.depth().";
        throw std::runtime_error(error);
    }

    if(mortarJointImage.size != image_and.size)
    {
        std::string error = "Error in ExtractMortarJoint(): mortarJointImage.size != maskImage.size.";
        throw std::runtime_error(error);
    }

    if(!mortarJointImage.data)
    {
        std::cout << "mortarJointImage is invalid." << std::endl;
        throw std::runtime_error("Error occur");
    }
    // std::cout << "mortarJointImage: " << mortarJointImage << std::endl;
    cv::imwrite("/home/ubuntu/tmp/mortarJointImage.png", mortarJointImage);
    // std::cout << "****----save mortarJointImage" << std::endl;
    cv::imwrite("/home/ubuntu/tmp/maskImage.png", maskImage);
    // std::cout << "****----save maskImage" << std::endl;
    // std::ofstream ofile("/home/ubuntu/tmp/tmp.txt", std::ofstream::out);
    // if(!ofile.is_open())
    // {
    //     std::cout << "error: tmp.txt is invalid" << std::endl;
    //     return 0;
    // }
    for(std::size_t row = 0; row < height; ++row)
    {
        const uchar* mortarJointImageData = mortarJointImage.ptr<const uchar>(row);
        const uchar* projectImageData = maskImage.ptr<const uchar>(row);
        uchar* image_and_Data = image_and.ptr<uchar>(row);
        for(std::size_t col = 0; col < width; ++col)
        {
            if(mortarJointImageData[col] && projectImageData[col])
            {
                cv::Point point;
                point.x = col;
                point.y = row;
                int index = 0;
                Search(point, index, ROW_MAJOR);  
                image_and_Data[col] = 255;
                indicesPtr -> indices.push_back(index);

                // ofile << "row: " << row << " col: " << col << " index: " << index << std::endl;
            }
        }
    }
    // ofile.close();
    cv::imwrite("/home/ubuntu/tmp/image_and.png", image_and);

    std::cout << "-------- end extract mortar joint" << std::endl;
    return 0;
}

pcl::ModelCoefficients Masonry::LeastSquaresPlaneFitting(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtr)
{
    std::cout << "plane fitting 1" << std::endl;
    float y2 = 0, yz = 0, y = 0, z2 = 0,  z = 0, xy = 0, xz = 0, x = 0;
    for(const auto& p : *pointCloudPtr)
    {
        y2 += p.y * p.y;
        yz += p.y * p.z;
        y += p.y;
        z2 += p.z * p.z;
        z += p.z;
        xy += p.x * p.y;
        xz += p.x * p.z;
        x += p.x;
    }
    std::cout << "plane fitting 2" << std::endl;

    Eigen::Matrix3d coefficients_matrix;
    coefficients_matrix << 
        y2, yz, y, 
        yz, z2, z, 
        y, z, pointCloudPtr->points.size();
    std::cout << "plane fitting 3" << std::endl;
    Eigen::Vector3d eq;
    eq << xy, xz, x;
    std::cout << "plane fitting 4" << std::endl;
    Eigen::Vector3d X = coefficients_matrix.colPivHouseholderQr().solve(eq);
    std::cout << "plane fitting 5" << std::endl;

    Eigen::Vector3d plane_normal_vector;
    plane_normal_vector << 1.0, -X[0], -X[1];
    plane_normal_vector /= plane_normal_vector.norm();
    float d = -X[2] / plane_normal_vector.norm();
    std::cout << "plane fitting 6" << std::endl;

    std::cout << "plane_normal_vector: " << plane_normal_vector.transpose() << std::endl;

    pcl::ModelCoefficients planeCoefficients;
    planeCoefficients.values.resize(4);
    auto& coeffs = planeCoefficients.values;
    coeffs[0] = plane_normal_vector[0];
    coeffs[1] = plane_normal_vector[1];
    coeffs[2] = plane_normal_vector[2];
    coeffs[3] = d;
    std::cout << "plane fitting 7" << std::endl;
    std::cout << "planeCoefficients: " << coeffs[0] << " " << coeffs[1] << " " << coeffs[2] << " " << coeffs[3]  << std::endl;
    return planeCoefficients;
}

pcl::ModelCoefficients Masonry::LeastSquaresQuadricPlaneFitting(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtr)
{
    float x = 0., y = 0.;
    float x2 = 0., xy = 0., y2 = 0.;
    float x3 = 0., x2y = 0., xy2 = 0., y3 = 0.;
    float x4 = 0., x3y = 0., x2y2 = 0., xy3 = 0., y4 = 0.;
    float z = 0., xz = 0., yz = 0., x2z = 0., xyz = 0., y2z = 0.;

    for(const auto& point : *pointCloudPtr)
    {
        float x_p = point.x;
        float y_p = point.y;
        float z_p = point.z;
        float x2_p = x_p * x_p;
        float y2_p = y_p * y_p;
        float x3_p = x2_p * x_p;
        float y3_p = y2_p * y_p;
        float x4_p = x3_p * x_p;
        float y4_p = y3_p * y_p;        

        x += x_p;
        y += y_p;

        x2 += x2_p;
        xy += x_p * y_p;
        y2 += y2_p;

        x3 += x3_p;
        x2y += x2_p * y_p;
        xy2 += x_p * y2_p;
        y3 += y3_p;

        x4 += x4_p;
        x3y += x3_p * y_p;
        x2y2 += x2_p * y2_p;
        xy3 += x_p * y3_p;
        y4 += y4_p;

        z += z_p;
        xz += x_p * z_p;
        yz += y_p * z_p;
        x2z += x2_p * z_p;
        xyz += x_p * y_p * z_p;
        y2z += y2_p * z_p;
    }

    float n = pointCloudPtr->size();
    Eigen::Matrix<float, 6, 6, Eigen::RowMajor> coefficients_matrix;
    coefficients_matrix << 
        n, x, y, x2, xy, y2,
        x, x2, xy, x3, x2y, xy2,
        y, xy, y2, x2y, xy2, y3,
        x2, x3, x2y, x4, x3y, x2y2,
        xy, x2y, xy2, x3y, x2y2, xy3,
        y2, xy2, y3, x2y2, xy3, y4;
    Eigen::Matrix<float, 6, 1> eq;
    eq << z, xz, yz, x2z, xyz, y2z;
    Eigen::Matrix<float, 6, 1> X = coefficients_matrix.colPivHouseholderQr().solve(eq);
    
    pcl::ModelCoefficients planeCoefficients;
    planeCoefficients.values.resize(6);
    auto& coeffs = planeCoefficients.values;
    coeffs[0] = X[0];
    coeffs[1] = X[1];
    coeffs[2] = X[2];
    coeffs[3] = X[3];
    coeffs[4] = X[4];
    coeffs[5] = X[5];

    std::cout << "planeCoefficients: ";
    for(const auto& coeff : coeffs)
    {
        std::cout << coeff << " ";
    } 
    std::cout << std::endl;
    return planeCoefficients;
}

int Masonry::MedianFilter(cv::Mat& src, cv::Mat& dst)
{
    cv::medianBlur(src, dst, 3);
    return 0;
}

int Masonry::DilateFilter(cv::Mat& src, cv::Mat& dst)
{
    cv::dilate(src, dst, cv::Mat());
    return 0;
}

int Masonry::SetImageWidth(const int& imageWidth)
{
    _imageWidth = imageWidth;

    return 0;
}

int Masonry::SetImageHeight(const int& imageHeight)
{
    _imageHeight = imageHeight;

    return 0;
}

}//namespace am
















    // else if(method == POINTCLOUD)
    // {
    //     /**计算点到平面的距离**/
    //     int status;
    //     // status = DistancePoint2Plane(cloud_optimal, minPoint, maxPoint);
    //     pcl::ModelCoefficients plane_coeffs;
    //     // plane_coeffs = cp.Plane(cloud_optimal, MASONRY);
    //     plane_coeffs = LeastSquaresPlaneFitting(cloud_optimal);

    //     std::ofstream of("../result/distance.txt");
    //     if(!of.is_open())
    //     {
    //         throw std::runtime_error("Error in Calculate(): distance.txt cannot open.");
    //     }


    //     for (auto & p : *cloud_optimal) 
    //     {
    //         float distance = pcl::pointToPlaneDistanceSigned<pcl::PointXYZI>(
    //             p,
    //             plane_coeffs.values[0],
    //             plane_coeffs.values[1],
    //             plane_coeffs.values[2],
    //             plane_coeffs.values[3]);

    //             p.intensity =  std::round(distance) + 127;

    //             of << std::round(distance) << std::endl;
    //     }

    //     of.close();


    //     FileIO ofile;
    //     ofile.SetProperty("../result/intensity.txt");
    //     ofile.Write(cloud_optimal);

    //     /** 生成二维图像 **/

    //     cv::Mat depthMap_origin(_imageHeight, _imageWidth, CV_8UC1, cv::Scalar::all(0));
    //     // status = ShowImage(depthMap_origin, "depthMap_origin");
    //     // std::cout << "test1" << std::endl;

    //     // std::cout << "image empty: " << depthMap_origin.data << std::endl;
    //     std::cout << "image width: " << depthMap_origin.cols << std::endl;
    //     std::cout << "image height: " << depthMap_origin.rows << std::endl;
    //     status = CreateDepthMap(cloud_optimal, index_optimal, depthMap_origin);
    //     std::cout << "status: " << status << std::endl;
    //     // std::cout << "image data: " << depthMap_origin.data << std::endl;
    //     status = ShowImage(depthMap_origin, "depthMap_origin");



    //     // cv::Mat detphMap_equalizeHist;
    //     // cv::equalizeHist(depthMap_origin, detphMap_equalizeHist);
    //     // status = ShowImage(detphMap_equalizeHist, "detphMap_equalizeHist");

    //     cv::Mat depthMap_medianFilter;
    //     // status = BilateraFilter(detphMap_equalizeHist, depthMap_bilateraFilter);
    //     status = MedianFilter(depthMap_origin, depthMap_medianFilter);
    //     status = ShowImage(depthMap_medianFilter, "depthMap_medianFilter");


    //     cv::Mat depthMap_histogram;
    //     std::vector<int> pixel;
    //     status = GetHistogramImage(depthMap_medianFilter, pixel, depthMap_histogram);
    //     status = ShowImage(depthMap_histogram, "depthMap_histogram");

    //     for(int  i = 0; i < pixel.size(); i++)
    //     {
    //         std::cout << "pixel[" << i << "]: " << pixel[i] << std::endl;
    //     }

    //     for(int i = 0 ; i < depthMap_medianFilter.rows; i++)
    //     {
    //         uchar* data = depthMap_medianFilter.ptr<uchar>(i);
    //         for(int j = 0; j < depthMap_medianFilter.cols; j++)
    //         {
    //             auto it = std::find(pixel.begin(), pixel.end(), data[j]);
    //             if( it != pixel.end() )
    //                 data[j] = 0;
    //             else
    //                 data[j] = 127;
    //         }
    //     }
    //     status = ShowImage(depthMap_medianFilter, "depthMap_medianFilter_pretrate");

    //     cv::Mat depthMap_sobel_x, depthMap_sobel_y;
    //     status = Sobel(depthMap_medianFilter, depthMap_sobel_x, ROW_MAJOR);
    //     status = Sobel(depthMap_medianFilter, depthMap_sobel_y, COL_MAJOR);
    //     status = ShowImage(depthMap_sobel_x, "depthMap_sobel_x");
    //     status = ShowImage(depthMap_sobel_y, "depthMap_sobel_y");

    //     return 0;
    // }
    // else if(method == POINTCLOUD_AND_IMAGE)
    // {

    // }
    // else
    // {

    // }





    // std::vector<cv::Point> point_x;
    // std::vector<cv::Point> point_y;
    // DetectEdge(depthMap_sobel_x, ROW_MAJOR, point_x);
    // DetectEdge(depthMap_sobel_y, COL_MAJOR, point_y);

    // std::vector<int> indices_x;
    // std::vector<int> indices_y;
    // CalculateIndex(point_x, indices_x, ROW_MAJOR);
    // CalculateIndex(point_y, indices_y, COL_MAJOR);

    // std::vector<int> indices_all;
    // indices_all += indices_x;
    // indices_all += indices_y;

    // /**删除重复的索引**/
    // sort(indices_all.begin(),indices_all.end());
    // indices_all.erase(unique(indices_all.begin(), indices_all.end()), indices_all.end());
    // std::cout << "indices_all.size() = " << indices_all.size () << std::endl;
    
    // /**根据索引删除点云中的离群点**/
	// pcl::PointIndices::Ptr outliners(new pcl::PointIndices(indices_all));
	// // outliners->indices.resize(vec_Bound1cm.size());
	// // for (size_t i = 0; i < vec_Bound1cm.size(); i++)
	// // {
	// // 	outliners->indices[i] = vec_Bound1cm[i];
	// // }

    // pcl::PointIndices::Ptr indices_final;
    // IndexFilter(index_optimal, outliners, indices_final);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter;
    // cp.Extract(cloud_optimal, indices_final,cloud_filter, true, false);

//     return 0;
// }



// int Masonry::CalculateIndex(std::vector<cv::Point>& points, std::vector<int>& indices, const int& method)
// {
//     indices.resize(points.size());
//     for(std::size_t i = 0; i < points.size(); i++)
//     {
//         auto point = points[i];
//         Search(point, indices[i], method);
//     }
//     return 0;
// }

// int Masonry::Find(const int& element, std::vector<int>& indices_optimal)
// {
//     std::vector<int>::iterator it;

//     it = find (indices_optimal.begin(), indices_optimal.end(), element);
//     if (it != indices_optimal.end())
//         return it - indices_optimal.begin();
//     else
//         return -1;
// }

// int Masonry::IndexFilter(pcl::PointIndices::Ptr& indices_optimal, pcl::PointIndices::Ptr& outliners, pcl::PointIndices::Ptr& indices_final)
// {
//     for(std::size_t i = 0; i < outliners->indices.size(); i++)
//     {
//         auto element = outliners->indices[i];
//         indices_final->indices.push_back(Find(element, indices_optimal->indices));
//     }
// }



// int Masonry::Sobel(cv::Mat& src, cv::Mat& dst, const int& orientation)
// {
//     switch(orientation)
//     {
//         case ROW_MAJOR:
//         {
//             cv::Sobel(src, dst, CV_16S, 1, 0, _ksize, _scale[_ksize]);
//             break;
//         }
//         case COL_MAJOR:
//         {
//             cv::Sobel(src, dst, CV_16S, 0, 1, _ksize, _scale[_ksize]);
//             break;
//         }
//         default:
//         {
//             std::string error = "Error in Sobel(): wrong orientation.";
//             int statusCode = -1;
//             throw std::runtime_error(error);
//         }
//     }
//     return 0;
// }

// int Masonry::DetectEdge(cv::Mat& img, const int& orientation, std::vector<cv::Point>& points)
// {
//     switch(orientation)
//     {
//         case ROW_MAJOR:
//         {
//             for (decltype(img.rows) r = 0; r < img.rows; ++r) 
//             {
//                 auto pRow = _dx.ptr<short>(r);  // NOLINT
//                 auto minmax = std::minmax_element(pRow, pRow + img.cols);

//                 auto minEle = minmax.first;
//                 auto maxEle = minmax.second;

//                 auto minVal = *minEle;
//                 auto maxVal = *maxEle;

//                 auto minPos = minEle - pRow;
//                 auto maxPos = maxEle - pRow;
//                 auto width = minPos - maxPos;

//                 // Filter by threshold
//                 if (maxVal < _threshold || minVal > -_threshold) {continue;}

//                 // Filter by width
//                 if (width < _widthMin || width > _widthMax) {continue;}

//                 // Filter by position
//                 if (maxPos <= 0 || minPos >= img.cols - 1) {continue;}                
//             }
//         }
//         case COL_MAJOR:
//         {

//         }
//     }

// }



// int Masonry::CreateDepthMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointIndices::Ptr indices, cv::Mat& depthMap)
// {
//     auto cloud_size = cloud->points.size();
//     auto indices_size = indices -> indices.size();
//     std::cout << "cloud_size: " << cloud_size << std::endl;
//     std::cout << "indices_size: " << indices_size << std::endl;

//     if(cloud_size != cloud_size)
//     {
//         std::string error = "Error in CreateDepthMap(): cloud_size != cloud_size.";
//         int statusCode = -1;
//         throw std::runtime_error(error);
//     }

//     for(std::size_t i = 0; i < cloud_size; i++)
//     {
//         auto index = indices->indices[i];
//         auto point = cloud->points[i];

//         cv::Point pixelCoordinate;
//         Search(index, pixelCoordinate, ROW_MAJOR);

//         depthMap.at<uchar>(pixelCoordinate.y, pixelCoordinate.x) = point.intensity;
//         // std::cout << "i: " << i << std::endl;
//     }

//     return 0;
// }

// int Masonry::GetHistogramImage(cv::Mat& src, std::vector<int>& pixel, cv::Mat& dst)
// {
//     Histogram1D hg;
//     dst = hg.getHistogramImage(src, pixel);
// }

// int Masonry::BilateraFilter(cv::Mat& src, cv::Mat& dst)
// {
//     cv::bilateralFilter(src, dst, 15, 20, 50);
//     return 0;
// }


// int Masonry::DistancePoint2Plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Vector3f& minPoint, Eigen::Vector3f maxPoint)
// {
//     pcl::KdTreeFLANN<pcl::PointXYZI> kdtree; //创建一个KdTreeFLANN对象，输入点云
//     kdtree.setInputCloud(cloud);

//     float deltaX = maxPoint[0] - minPoint[0];
//     float deltaY = maxPoint[1] - minPoint[1];
//     float deltaZ = maxPoint[2] - minPoint[2];

//     std::vector<float> range;
//     range.push_back(deltaX);
//     range.push_back(deltaY);
//     range.push_back(deltaZ);

//     std::sort(range.begin(), range.end(), [&](const float& a, const float& b) -> bool {return a > b;});

//     float maxRange = range[0];
//     float mediumRange = range[1];
//     float minRange = range[2];

//     int meta = 5;
//     float edgeLength = maxRange / meta;
//     float radius = 1.8 * edgeLength;

//     for(int i = 0; i < meta; i++)
//     {
//         for(int j = 0; j < meta; j++)
//         {
//             for(int k = 0; k < meta; k++)
//             {
//                 pcl::PointXYZI centerPoint;
//                 centerPoint.x = minPoint[0] + i * edgeLength + edgeLength / 2.0;
//                 centerPoint.y = minPoint[1] + j * edgeLength + edgeLength / 2.0;
//                 centerPoint.z = minPoint[2] + k * edgeLength + edgeLength / 2.0;
//                 centerPoint.intensity = 0.;

//                 pcl::PointIndices::Ptr k_indices(new pcl::PointIndices);
//                 std::vector<float> k_sqr_distance;

//                 if (kdtree.radiusSearch(centerPoint, radius, k_indices->indices, k_sqr_distance) > 0)
//                 {
//                     CloudPretreatment cp;
//                     pcl::PointCloud<pcl::PointXYZI>::Ptr k_points(new pcl::PointCloud<pcl::PointXYZI>);
//                     cp.Extract(cloud, k_indices, k_points, false);

//                     if(k_points -> points.size() < 1000)
//                         continue;
                    
//                     pcl::ModelCoefficients plane_coeffs;
//                     plane_coeffs = cp.Plane(k_points, MASONRY);

//                     for (auto & p : *k_points) 
//                     {
//                         float distance = pcl::pointToPlaneDistanceSigned<pcl::PointXYZI>(
//                             p,
//                             plane_coeffs.values[0],
//                             plane_coeffs.values[1],
//                             plane_coeffs.values[2],
//                             plane_coeffs.values[3]);

//                         p.intensity = std::fabs( std::round(distance) );
//                     }
//                 }
//             }
//         }
//     }
// }
