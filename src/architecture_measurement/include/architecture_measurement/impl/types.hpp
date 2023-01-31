#pragma once
#ifndef _TYPES_H
#define _TYPES_H

#include <iostream>

#include "Eigen/Core"

namespace am 
{

class Point3d
{
    public:
        double x;
        double y;
        double z;
        Eigen::Vector3d point;

    public:
        Point3d();
        Point3d(const double& _x,const double& _y,const double& _z);
        Point3d(const Eigen::Vector3d& _point);
        Point3d(const Point3d& _point);
        // Point3d(Point3d&& _point);
        ~Point3d();
        Point3d& operator=(const Point3d& _point);
        friend std::ostream& operator<<(std::ostream& os, const Point3d& point);
};

class Vector3d
{
    public:
        double x;
        double y;
        double z;
        Eigen::Vector3d vector;
    public:
        Vector3d();
        Vector3d(const double& _x, const double& _y, const double& _z);
        Vector3d(const Eigen::Vector3d& _vector);
        Vector3d(const Vector3d& _vector);
        // Vector3d(Vector3d&& _vector);
        ~Vector3d();
        Vector3d& operator=(const Vector3d& _vector);
        friend std::ostream& operator<<(std::ostream& os, const Vector3d& vector);

        void Normalization(const bool& normlize);
};

class Line3d
{
    public:
        Point3d point;
        Vector3d vector;
    public:
        Line3d();
        Line3d(const Point3d& _point, const Vector3d& _vector);
        Line3d(const Line3d& _line);
        // Line3d(Line3d&& _line);
        ~Line3d();
        Line3d& operator=(const Line3d& _line);
        friend std::ostream& operator<<(std::ostream& os, const Line3d& line);
};

class Plane3d
{
    public:
        double a;
        double b;
        double c;
        double d;
        Eigen::Vector4d plane;

    public:
        Plane3d();
        Plane3d(const double& _a, const double& _b, const double& _c, const double& _d);
        Plane3d(const Plane3d& _plane);
        // Plane3d(Plane3d&& _plane);  
        ~Plane3d();
        Plane3d& operator=(const Plane3d& _plane);
        friend std::ostream& operator<<(std::ostream& os, const Plane3d& plane);
};

Plane3d ConvertToPlane(const Line3d& line);

Line3d ConvertToLine(const Plane3d& plane);

}//namespace am



#endif //_TYPES_H

