#pragma once
#ifndef _SPACE_ANALYTIC_GEOMETRY_H
#define _SPACE_ANALYTIC_GEOMETRY_H

#include <iostream>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include <pcl/pcl_macros.h>

const double INFINITESMAL = 1e-4;

class Point3d
{
public:
    double x;
    double y;
    double z;
    Eigen::Vector3d point;

public:
    Point3d();
    Point3d(const double& _x, const double& _y, const double& _z);
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

class DoublePoints
{
private:
    Point3d startPoint;
    Point3d endPoint;

public:
    DoublePoints();
    DoublePoints(const Point3d& _startPoint, const Point3d& _endPoint);
    ~DoublePoints();

    DoublePoints& operator=(const DoublePoints& dp);
    void SetProperty(const Point3d& _startPoint, const Point3d& _endPoint);

    void GetVector(Vector3d& vector);
    void GetDistance(double& distance);
    void GetLine(Line3d& line);
};

class DoubleLines
{
private:
    Line3d line1;
    Line3d line2;

public:
    DoubleLines();
    DoubleLines(const Line3d& _line1, const Line3d& _line2);
    ~DoubleLines();

    DoubleLines& operator=(const DoubleLines& dl);
    void SetProperty(const Line3d& _line1, const Line3d& _line2);

    bool IsCoplanar();
    bool IsCoincident();
    bool IsParallel();
    bool IsVertical();
    bool IsIntersectant();
    void GetDistance(double& distance);
    void GetIntersectionPoint(Point3d& point);
    void GetCommonNormalLine(Line3d& line);
};

class DoublePlanes
{
private:
    Plane3d plane1;
    Plane3d plane2;

public:
    DoublePlanes();
    DoublePlanes(const Plane3d& _plane1, const Plane3d& _plane2);
    ~DoublePlanes();

    DoublePlanes& operator=(const DoublePlanes& dp);
    void SetProperty(const Plane3d& plane1, const Plane3d& plane2);

    bool IsParallel();
    bool IsVertical();
    void GetDistance(double& distance);
    void GetIntersectionLine(Line3d& line);
};

class PointAndLine
{
private:
    Point3d point;
    Line3d line;

public:
    PointAndLine();
    PointAndLine(const Point3d& _point, const Line3d& _line);
    ~PointAndLine();

    PointAndLine& operator=(const PointAndLine& pal);
    void SetProperty(const Point3d& _point, const Line3d& _line);

    bool IsIncluded();
    void GetProjection(Point3d& _point);
    void GetDistance(double& diatance);

};

class PointAndPlane
{
private:
    Point3d point;
    Plane3d plane;

public:
    PointAndPlane();
    PointAndPlane(const Point3d& _point, const Plane3d& _plane);
    ~PointAndPlane();

    PointAndPlane& operator=(const PointAndPlane& pap);
    void SetProperty(const Point3d& _point, const Plane3d& _plane);

    bool IsIncluded();
    void GetDistance(double& distance);
    void GetProjection(Point3d& point);

};


class LineAndPlane
{
private:
    Line3d line;
    Plane3d plane;


public:
    LineAndPlane();
    LineAndPlane(const Line3d& _line, const Plane3d& _plane);
    ~LineAndPlane();

    LineAndPlane& operator=(const LineAndPlane& lap);
    void SetProperty(const Line3d& _line, const Plane3d& _plane);

    bool IsIncluded();
    bool IsVertical();
    bool IsParallel();
    void GetAngle(double& angle);
    void GetProjection(Line3d& _line);
    void GetIntersectionPoint(Point3d& point);
};

#endif //_SPACE_ANALYTIC_GEOMETRY_H