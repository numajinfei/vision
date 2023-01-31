#include "architecture_measurement/impl/inclinometer.hpp"

Inclinometer::Inclinometer() {}

Inclinometer::Inclinometer(const float& angleX, const float& angleY) : 
    _angleX(angleX), _angleY(angleY) {}

Inclinometer::~Inclinometer() {}

int Inclinometer::GetMatrix(Eigen::Matrix3f& rotation)
{
    Eigen::Vector3f axisX{std::cos(_angleX), 0, -std::sin(_angleX)};
    Eigen::Vector3f axisY{std::sin(_angleY) * std::tan(_angleX), std::sqrt( 1 - std::pow( std::sin(_angleY),2) * ( 1 + std::pow(std::tan(_angleX), 2) ) ), std::sin(_angleY)};
    Eigen::Vector3f axisZ = axisX.cross(axisY);

    rotation.col(0) = axisX;
    rotation.col(1) = axisY;
    rotation.col(2) = axisZ;

    return 0;
}

int Inclinometer::SetAngleXY(const float& angleX, const float& angleY)
{
    _angleX = angleX / 180 * M_PI;
    _angleY = angleY / 180 * M_PI;

    return 0;
}

int Inclinometer::GetAngleXY(float& angleX, float& angleY)
{
    angleX = _angleX;
    angleY = _angleY;
    return 0;
}