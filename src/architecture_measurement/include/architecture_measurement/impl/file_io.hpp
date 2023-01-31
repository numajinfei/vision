#pragma once
#ifndef _FILE_IO_H
#define _FILE_IO_H

#include <string>

#include "pcl/point_types.h"
#include "pcl/common/distances.h"

namespace am
{

class FileIO
{
    private:
        std::string fileName;

    public:
        FileIO();
        FileIO(const std::string& _fileName);
        FileIO(const FileIO& _fileIO) = delete;
        // FileIO(FileIO&& _fileIO);
        ~FileIO();

        void SetProperty(const std::string& _fileName);

        int Read(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
        int Write(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
};

}//namespace am

#endif //_FILE_IO_H