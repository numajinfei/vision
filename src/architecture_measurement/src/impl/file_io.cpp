#include "architecture_measurement/impl/file_io.hpp"

#include <fstream>
#include <sstream>
#include <exception>

#include "pcl/io/pcd_io.h"

namespace am
{
constexpr unsigned int CAPACITY = 3200000;

FileIO::FileIO()
{

}

FileIO::FileIO(const std::string& _fileName) : fileName(_fileName)
{

}

FileIO::~FileIO()
{

}

void FileIO::SetProperty(const std::string& _fileName)
{
    fileName = _fileName;
}


int FileIO::Read(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    std::size_t found = fileName.find_last_of(".");
    std::string postfix =  fileName.substr(found+1);

    if(postfix == "txt")
    {
        std::ifstream ifile(fileName, std::ifstream::in);
        if(!ifile.is_open())
            return -1;

        cloud->points.reserve(CAPACITY);

        while(!ifile.eof())
        {
            float x, y, z, intensity;
            ifile >> x  >> y  >> z >> intensity;
            pcl::PointXYZI point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.intensity = intensity;
            cloud->push_back(point);
        }
        ifile.close();  
    }
    else if(postfix == "pcd")
    {            
        if (pcl::io::loadPCDFile<pcl::PointXYZI> (fileName, *cloud) == -1)
            return -1;
    } 
    return 0;
}   

int FileIO::Write(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    std::ofstream ofile(fileName,std::ofstream::out);
    if(!ofile.is_open())
        return -1;
    
    ofile << std::fixed;
    for (const auto & p : *cloud) 
    {
        ofile 
            << p.x  << ' ' 
            << p.y  << ' ' 
            << p.z  << ' ' 
            << p.intensity << '\n';
    }

    ofile.close();

    return 0;
}

}//namespace am
