#pragma once
#ifndef PARAMS_CONFIG_H
#define PARAMS_CONFIG_H

#include "yaml-cpp/yaml.h"
#include "Global.h"
#include <iostream>

class Params
{
    private:
        YAML::Node node;   

    public:
        Params();
        ~Params();

        static bool LoadTransformParams(const std::string &configPath, std::shared_ptr<TransformParams> &ParamsPtr);
        static bool LoadPointCloudParams(const std::string &configPath, std::shared_ptr<PointCloudParams> &ParamsPtr);
};

#endif  //PARAMS_CONFIG_H