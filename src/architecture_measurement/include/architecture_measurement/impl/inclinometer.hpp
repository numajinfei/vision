#pragma once
#ifndef _INCLINOMETER_H
#define _INCLINOMETER_H

#include <iostream>
#include <cmath>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

/**
 * @brief 
 * 
 */
class Inclinometer
{
    public:

        /**
         * @brief Construct a new Inclinometer object
         * 
         */
        Inclinometer();

        /**
         * @brief Construct a new Inclinometer object
         * 
         * @param angleX 
         * @param angleY 
         */
        Inclinometer(const float& angleX, const float& angleY);

        /**
         * @brief Destroy the Inclinometer object
         * 
         */
        ~Inclinometer();

        /**
         * @brief Get the Matrix object
         * 
         * @return int 
         */
        int GetMatrix(Eigen::Matrix3f& rotation);

        /**
         * @brief Set the Property object
         * 
         * @param angleX 
         * @param angleY 
         * @return int 
         */
        int SetAngleXY(const float& angleX, const float& angleY);

        int GetAngleXY(float& angleX, float& angleY);

    private:
        float _angleX; /**< */
        float _angleY; /**< */
};

#endif //_INCLINOMETER_H