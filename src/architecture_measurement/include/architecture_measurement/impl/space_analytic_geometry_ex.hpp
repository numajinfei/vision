
#pragma once
#ifndef _SPACE_ANALYTIC_GEOMETRY_EX_H
#define _SPACE_ANALYTIC_GEOMETRY_EX_H

#include "architecture_measurement/impl/types_ex.hpp"

#include <cstdlib>
#include <string>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

/**
 * @brief namespace architecture measurement. 
 * Implementation of measurement items.
 * 
 */
namespace am 
{

constexpr float INFINITESMAL = 1e-4;

template <typename Type>
class DoublePoints
{
    public:
        Point3D<Type> startPoint;/**< endpoint1 */  
        Point3D<Type> endPoint;/**< endpoint2 */
        
    public:
        /**
         * @brief Construct a new Double Points object
         * 
         */
        DoublePoints();

        /**
         * @brief Construct a new Double Points object
         * 
         * @param _startPoint endpoint1
         * @param _endPoint endpoint2
         */
        DoublePoints(const Point3D<Type>& _startPoint, const Point3D<Type>& _endPoint);

        DoublePoints(const DoublePoints<Type>& _doublePoints);

        DoublePoints(DoublePoints<Type>&& _doublePoints);

        /**
         * @brief Destroy the Double Points object
         * 
         */
        ~DoublePoints();

        /**
         * @brief Assignment Operators
         * 
         * @param dp a Double Points object
         * @return DoublePoints& a Double Points object
         */
        DoublePoints<Type>& operator=(const DoublePoints<Type>& _doublePoints);

        /**
         * @brief Get the Vector object
         * 
         * @param vector Vector between two points
         */
        void GetVector(Vector3D<Type>& _vector);     

        /**
         * @brief Get the Distance object
         * 
         * @param distance distance between two points
         * @return * void 
         */
        void GetDistance(Type& _distance);

        /**
         * @brief Get the Line object
         * 
         * @param line a Line object between two points
         */
        void GetLine(Line3D<Type>& _line);
};


template <typename Type>
class DoubleLines
{
    public:
        Line3D<Type> line1;/**< line1 */
        Line3D<Type> line2;/**< line2 */

    public:
        /**
         * @brief Construct a new Double Lines object
         * 
         */
        DoubleLines();
        
        /**
         * @brief Construct a new Double Lines object
         * 
         * @param _line1 a Line object
         * @param _line2 a Line object
         */
        DoubleLines(const Line3D<Type>& _line1, const Line3D<Type>& _line2);

        DoubleLines(const DoubleLines<Type>& _doubleLines);

        DoubleLines(DoubleLines<Type>&& _doubleLines);

        /**
         * @brief Destroy the Double Lines object
         * 
         */
        ~DoubleLines();

        /**
         * @brief Assignment operator
         * 
         * @param dl a Double Line object
         * @return DoubleLines& 
         */
        DoubleLines<Type>& operator=(const DoubleLines<Type>& dl);

        /**
         * @brief is two lines coplanar
         * 
         * @return true 
         * @return false 
         */
        bool IsCoplanar();

        /**
         * @brief is two lines coincident
         * 
         * @return true 
         * @return false 
         */
        bool IsCoincident();

        /**
         * @brief is two lines parallel
         * 
         * @return true 
         * @return false 
         */
        bool IsParallel();

        /**
         * @brief is two lines vertical
         * 
         * @return true 
         * @return false 
         */
        bool IsVertical();

        /**
         * @brief is two lines intersectant
         * 
         * @return true 
         * @return false 
         */
        bool IsIntersectant();

        /**
         * @brief Get the Distance object, if two lines are parallel
         * 
         * @param distance 
         */
        void GetDistance(Type& distance);

        /**
         * @brief Get the Intersection Point object, if two lines are intersectant
         * 
         * @param point intersectant point
         */
        void GetIntersectionPoint(Point3D<Type>& point);

        /**
         * @brief Get the Common Normal Line object, if two lines are not parallel or intersectant
         * 
         * @param line 
         */
        void GetCommonNormalLine(Line3D<Type>& line);        
};

template <typename Type>
class DoublePlanes
{
    public:
        Plane3D<Type> plane1;/**< plane1 */
        Plane3D<Type> plane2;/**< plane2 */

    public:
    /**
     * @brief Construct a new Double Planes object
     * 
     */
    DoublePlanes();

    /**
     * @brief Construct a new Double Planes object
     * 
     * @param _plane1 plane1
     * @param _plane2 plane2
     */
    DoublePlanes(const Plane3D<Type>& _plane1, const Plane3D<Type>& _plane2);

    DoublePlanes(const DoublePlanes<Type>& _doublePlanes);

    DoublePlanes(DoublePlanes<Type>&& _doublePlanes);

    /**
     * @brief Destroy the Double Planes object
     * 
     */
    virtual ~DoublePlanes();

    /**
     * @brief Assignment operator
     * 
     * @param dp a Double Planes object
     * @return DoublePlanes& a Double Planes object
     */
    DoublePlanes<Type>& operator=(const DoublePlanes<Type>& dp);


    /**
     * @brief is two planes are parallel
     * 
     * @return true 
     * @return false 
     */
    bool IsParallel();

    /**
     * @brief is two planes vertical
     * 
     * @return true 
     * @return false 
     */
    bool IsVertical();

    /**
     * @brief Get the Distance object, if two planes are parallel
     * 
     * @param distance 
     */
    void GetDistance(Type& distance);

    /**
     * @brief Get the Intersection Line object, if two planes are intersectant
     * 
     * @param line 
     */
    void GetIntersectionLine(Line3D<Type>& line);
};


template <typename Type>
class PointAndLine
{
    private:
        Point3D<Type> point;/**< point */
        Line3D<Type> line;/**< line */

    public:
        /**
         * @brief Construct a new Point And Line object
         * 
         */
        PointAndLine();

        /**
         * @brief Construct a new Point And Line object, 
         * 
         * @param _point point
         * @param _line line
         */
        PointAndLine(const Point3D<Type>& _point, const Line3D<Type>& _line);

        PointAndLine(const PointAndLine& _pointAndLine);

        PointAndLine(PointAndLine&& _pointAndLine);

        /**
         * @brief Destroy the Point And Line object
         * 
         */
        ~PointAndLine();

        /**
         * @brief assignment operator
         * 
         * @param pal a Point And Line object
         * @return PointAndLine& a Point And Line object
         */
        PointAndLine& operator=(const PointAndLine& pal);

        /**
         * @brief if is point on line
         * 
         * @return true 
         * @return false 
         */
        bool IsIncluded();
        void GetProjection(Point3D<Type>& _point);
        void GetDistance(Type& diatance);

};

template <typename Type>
class PointAndPlane
{
    private:
        Point3D<Type> point;/**< point */
        Plane3D<Type> plane;/**< plane */
    
    public:
        /**
         * @brief Construct a new Point And Plane object
         * 
         */
        PointAndPlane();

        /**
         * @brief Construct a new Point And Plane object
         * 
         * @param _point point
         * @param _plane plane
         */
        PointAndPlane(const Point3D<Type>& _point, const Plane3D<Type>& _plane);

        PointAndPlane(const PointAndPlane& _pointAndPlane);

        PointAndPlane(PointAndPlane&& _pointAndPlane);

        /**
         * @brief Destroy the Point And Plane object
         * 
         */
        ~PointAndPlane();

        /**
         * @brief assignment operator
         * 
         * @param pap a Point And Plane object
         * @return PointAndPlane& 
         */
        PointAndPlane& operator=(const PointAndPlane& pap);
        
        /**
         * @brief Set the Property object
         * 
         * @param _point ///point
         * @param _plane ///plane
         */
        void SetProperty(const Point3D<Type>& _point, const Plane3D<Type>& _plane);

        /**
         * @brief if is point on plane
         * 
         * @return true 
         * @return false 
         */
        bool IsIncluded();

        /**
         * @brief Get the Distance object
         * 
         * @param distance distance between point and plane
         */
        void GetDistance(Type& distance);

        /**
         * @brief Get the Projection object
         * 
         * @param point project point
         */
        void GetProjection(Point3D<Type>& point);
};

template <typename Type>
class LineAndPlane
{
    private:
        Line3D<Type> line;/**< line */
        Plane3D<Type> plane;/**< plane */

    public:
        /**
         * @brief Construct a new Line And Plane object
         * 
         */
        LineAndPlane();

        /**
         * @brief Construct a new Line And Plane object
         * 
         * @param _line a Line object
         * @param _plane a Plane object
         */
        LineAndPlane(const Line3D<Type>& _line, const Plane3D<Type>& _plane);

        LineAndPlane(const LineAndPlane<Type>& _lineAndPlane);

        LineAndPlane(LineAndPlane&& _lineAndPlane);

        /**
         * @brief Destroy the Line And Plane object
         * 
         */
        ~LineAndPlane();

        /**
         * @brief assignment operator
         * 
         * @param lap a Line And Plane object
         * @return LineAndPlane& a Line And Plane object
         */
        LineAndPlane& operator=(const LineAndPlane& lap);

        /**
         * @brief if is line on plane
         * 
         * @return true 
         * @return false 
         */
        bool IsIncluded();

        /**
         * @brief is line perpendicularity to the plane 
         * 
         * @return true 
         * @return false 
         */
        bool IsVertical();

        /**
         * @brief is line parallel to the plane
         * 
         * @return true 
         * @return false 
         */
        bool IsParallel();

        /**
         * @brief Get the Angle object, if the line and plane are intersectant
         * 
         * @param angle angle between line and plane
         */
        void GetAngle(Type& angle);

        /**
         * @brief Get the Projection object
         * 
         * @param _line project line
         */
        void GetProjection(Line3D<Type>& _line);

        /**
         * @brief Get the Intersection Point object
         * 
         * @param point intersection point on plane
         */
        void GetIntersectionPoint(Point3D<Type>& point);
};

/** DoublePoints **/
template <typename Type>
DoublePoints<Type>::DoublePoints() {}

template <typename Type>
DoublePoints<Type>::DoublePoints(const Point3D<Type>& _startPoint, const Point3D<Type>& _endPoint) : 
    startPoint(_startPoint), endPoint(_endPoint) {}

template <typename Type>
DoublePoints<Type>::DoublePoints(const DoublePoints<Type>& _doublePoints)
{
    startPoint = _doublePoints.startPoint;
    endPoint = _doublePoints.endPoint;
}

template <typename Type>
DoublePoints<Type>::DoublePoints(DoublePoints<Type>&& _doublePoints)
{
    startPoint = _doublePoints.startPoint;
    endPoint = _doublePoints.endPoint;
}

template <typename Type>
DoublePoints<Type>::~DoublePoints() {}

template <typename Type>
DoublePoints<Type>& DoublePoints<Type>::operator=(const DoublePoints<Type>& _doublePoints)
{
    if(this == &_doublePoints)
        return *this;

    startPoint = _doublePoints.startPoint;
    endPoint = _doublePoints.endPoint;   

    return *this; 
}

template <typename Type>
void DoublePoints<Type>::GetVector(Vector3D<Type>& _vector)
{
    Vector3D<Type> vectorTmp{endPoint.point - startPoint.point};
    _vector = vectorTmp;
}

template <typename Type>
void DoublePoints<Type>::GetDistance(Type& _distance)
{
    Vector3D<Type> startToEndVector;
    GetVector(startToEndVector);
    _distance = startToEndVector.vector.norm();
}

template <typename Type>
void DoublePoints<Type>::GetLine(Line3D<Type>& _line)
{
    Point3D<Type> point{startPoint};
    Vector3D<Type> vector;
    GetVector(vector);
    Line3D<Type> lineTmp{point,vector};
    _line = lineTmp;
}


/** DoubleLines **/
template <typename Type>
DoubleLines<Type>::DoubleLines() {}

template <typename Type>
DoubleLines<Type>::DoubleLines(const Line3D<Type>& _line1, const Line3D<Type>& _line2) : 
    line1(_line1), line2(_line2) {}

template <typename Type>
DoubleLines<Type>::DoubleLines(const DoubleLines<Type>& _doubleLines)
{
    line1 = _doubleLines.line1;
    line2 = _doubleLines.line2;
}

template <typename Type>
DoubleLines<Type>::DoubleLines(DoubleLines<Type>&& _doubleLines)
{
    line1 = _doubleLines.line1;
    line2 = _doubleLines.line2;
}

template <typename Type>
DoubleLines<Type>::~DoubleLines() {}

template <typename Type>
DoubleLines<Type>& DoubleLines<Type>::operator=(const DoubleLines<Type>& _doubleLines)
{
    if(this == &_doubleLines)
        return *this;

    line1 = _doubleLines.line1;
    line2 = _doubleLines.line2;

    return *this;
}

template <typename Type>
bool DoubleLines<Type>::IsCoincident()
{   
    auto vector1 = line1.vector.vector;
    auto vector2 = line2.vector.vector;

    Type dev = (vector1.cross(vector2)).norm();
    if(dev < INFINITESMAL)
    {
        Type distance;
        DoublePoints<Type> dp(line1.point,line2.point);
        dp.GetDistance(distance);
        if(distance < INFINITESMAL)
            return true;
        else
            return false;
    }        
    else
        return false;
}

template <typename Type>
bool DoubleLines<Type>::IsParallel()
{    
    auto vec1 = line1.vector.vector;
    auto vec2 = line2.vector.vector;
    Type dev = (vec1.cross(vec2)).norm();
    if(dev < INFINITESMAL && !IsCoincident())
        return true;
    else
        return false;
}

template <typename Type>
bool DoubleLines<Type>::IsVertical()
{
    auto vector1 = line1.vector.vector;
    auto vector2 = line2.vector.vector;
    auto innerProduct = std::fabs(vector1.dot(vector2));
    if(innerProduct < INFINITESMAL)
        return true;
    else    
        return false;
}

template <typename Type>
bool DoubleLines<Type>::IsCoplanar()
{
    if(IsParallel() || IsIntersectant())
        return true;
    else
        return false;
}

template <typename Type>
bool DoubleLines<Type>::IsIntersectant()
{
    if(IsParallel())
        return false;

    const auto& vector1 = line1.vector.vector;
    const auto& vector2 = line2.vector.vector;
    const auto& vector3 = line1.point.point - line2.point.point;
    Eigen::Matrix<Type, 3, 3> paramMatrix;
    paramMatrix << vector1, vector2, vector3;
    if(std::fabs(paramMatrix.determinant()) < INFINITESMAL)
        return true;
    else
        return false;
}

template <typename Type>
void DoubleLines<Type>::GetDistance(Type& distance)
{
    if(IsParallel())
    {
        auto pointToPointVector = line1.point.point - line2.point.point;
        auto crossVector = pointToPointVector.cross(line1.vector.vector);
        distance = crossVector.norm();
    }
    else if(!IsCoplanar())
    {
        auto crossVector = line1.vector.vector.cross(line2.vector.vector);
        auto pointToPointVecxtor = line1.point.point - line2.point.point;
        distance = std::fabs(crossVector.dot(pointToPointVecxtor))/crossVector.norm();
    }
    else
        distance = 0.;
}

template <typename Type>
void DoubleLines<Type>::GetIntersectionPoint(Point3D<Type>& point)
{
    if(!IsIntersectant())
    {
        std::string error = "Error in GetIntersectionPoint(): no joint point.";
        throw std::runtime_error(error);
    }

    const auto& vector1 = line1.vector.vector;
    const auto& vector2 = line2.vector.vector;
    const auto& pointToPointVector = line2.point.point - line1.point.point;
    Eigen::Matrix<Type,3,2> A;
    A << vector1,vector2;
    Eigen::Matrix<Type,3,1> b;
    b << pointToPointVector;
    Eigen::Matrix<Type, 2, 1> x = A.colPivHouseholderQr().solve(b);

    const auto& point1 = line1.point.point;
    const auto& point2 = line2.point.point;
    auto intersectionPoint = point1 + x(0) * vector1;
    auto intersectionPoint_test = point2 + x(1) * vector2;

	point = Point3D<Type>(intersectionPoint);
}

template <typename Type>
void DoubleLines<Type>::GetCommonNormalLine(Line3D<Type>& line)
{
    auto op1 = line1.point.point;
    auto op2 = line2.point.point;
    auto b = op2 - op1;

    auto vector1 = line1.vector.vector;
    auto vector2 = line2.vector.vector;
    auto crossVector = vector1.cross(vector2);

    Eigen::Matrix<Type, 3, 3> A;
    A << vector1, -vector2, crossVector;
    auto x = A.colPivHouseholderQr().solve(b);

    auto normalLinePoint = op1 + x(0)*vector1;
    Point3D<Type> point{normalLinePoint};
    Vector3D<Type> vector{crossVector};
    Line3D<Type> lineTmp{point,vector};
    line = lineTmp;
}

/**** DoublePlanes ****/

template <typename Type>
DoublePlanes<Type>::DoublePlanes() {}

template <typename Type>
DoublePlanes<Type>::DoublePlanes(const Plane3D<Type>& _plane1, const Plane3D<Type>& _plane2) : 
    plane1(_plane1), plane2(_plane2) {}

template <typename Type>
DoublePlanes<Type>::DoublePlanes(const DoublePlanes<Type>& _doublePlanes)
{
    plane1 = _doublePlanes.plane1;
    plane2 = _doublePlanes.plane2;
}

template <typename Type>
DoublePlanes<Type>::DoublePlanes(DoublePlanes<Type>&& _doublePlanes)
{
    plane1 = _doublePlanes.plane1;
    plane2 = _doublePlanes.plane2;
}

template <typename Type>
DoublePlanes<Type>::~DoublePlanes() {}

template <typename Type>
DoublePlanes<Type>& DoublePlanes<Type>::operator=(const DoublePlanes<Type>& _doublePlanes)
{
    if(this == &_doublePlanes)
        return * this;

    plane1 = _doublePlanes.plane1;
    plane2 = _doublePlanes.plane2;

    return *this;
}

template <typename Type>
bool DoublePlanes<Type>::IsParallel()
{
    const Eigen::Matrix<Type, 3, 1> normalVector1{plane1.a,plane1.b,plane1.c};
    const Eigen::Matrix<Type, 3, 1> normalVector2{plane2.a,plane2.b,plane2.c};

    auto crossProductNorm = normalVector1.cross(normalVector2);
    if(crossProductNorm.norm() < INFINITESMAL)
        return true;
    else
        return false;
}

template <typename Type>
bool DoublePlanes<Type>::IsVertical()
{
    const auto normalVector1{plane1.a,plane1.b,plane1.c};
    const auto normalVector2{plane2.a,plane2.b,plane2.c};

    auto dotProductValue = std::fabs(normalVector1.dot(normalVector2));
    if(dotProductValue < INFINITESMAL)
        return true;
    else
        return false;
}

template <typename Type>
void DoublePlanes<Type>::GetDistance(Type& distance)
{
    if(!IsParallel())
    {
        std::string error = "Error in GetDistance(): planes are not parallel.";
        throw std::runtime_error(error);
    }

    const auto normalVector{plane1.a,plane1.b,plane1.c};
    auto mol = fabs(plane1.d - plane2.d);
    auto den = normalVector.norm();
    distance = mol / den;
}

template <typename Type>
void DoublePlanes<Type>::GetIntersectionLine(Line3D<Type>& line)
{
    if(IsParallel())
    {
        std::string error = "Error in GetIntersectionLine(): planes are parallel.";
        throw std::runtime_error(error);
    }

    const Eigen::Matrix<Type, 3, 1> normalVector1{plane1.a,plane1.b,plane1.c};
    const Eigen::Matrix<Type, 3, 1> normalVector2{plane2.a,plane2.b,plane2.c};

    Eigen::Matrix<Type, 2, 2> A;
    A << normalVector1.squaredNorm(), normalVector1.dot(normalVector2), normalVector1.dot(normalVector2), normalVector2.squaredNorm();
    Eigen::Matrix<Type, 2, 1> b;
    b << -plane1.d, -plane2.d;

    Eigen::Matrix<Type, 2, 1> x = A.inverse()*b;

    Point3D<Type> point{x(0) * normalVector1 + x(1) * normalVector2};
    Vector3D<Type> vector{normalVector1.cross(normalVector2)};
    Line3D<Type> lineTmp{point, vector};
    line = lineTmp;
}

/**** PointAndLine ****/
template <typename Type>
PointAndLine<Type>::PointAndLine() {}

template <typename Type>
PointAndLine<Type>::PointAndLine(const Point3D<Type>& _point, const Line3D<Type>& _line) : 
    point(_point), line(_line) {}

template <typename Type>
PointAndLine<Type>::PointAndLine(const PointAndLine& _pointAndLine)
{
    point = _pointAndLine.point;
    line = _pointAndLine.line;
}

template <typename Type>
PointAndLine<Type>::PointAndLine(PointAndLine&& _pointAndLine)
{
    point = _pointAndLine.point;
    line = _pointAndLine.line;
}

template <typename Type>
PointAndLine<Type>::~PointAndLine() {}

template <typename Type>
PointAndLine<Type>& PointAndLine<Type>::operator=(const PointAndLine<Type>& _pointAndLine)
{
    if(this == &_pointAndLine)
        return *this;

    point = _pointAndLine.point;
    line = _pointAndLine.line;

    return *this;
}

template <typename Type>
bool PointAndLine<Type>::IsIncluded()
{
    auto pointToPointVector = point.point - line.point.point;
    auto crossVector = pointToPointVector.cross(line.vector.vector);
    if(crossVector.norm() < INFINITESMAL)
        return true;
    else
        return false;

}

template <typename Type>
void PointAndLine<Type>::GetProjection(Point3D<Type>& _point)
{
    auto pointToPointVector = point.point - line.point.point;
    auto vector = line.vector.vector;

    auto t = pointToPointVector.dot(vector) / vector.squaredNorm();

    auto xd = vector(0) * t + line.point.x;
    auto yd = vector(1) * t + line.point.y;
    auto zd = vector(2) * t + line.point.z;

    Eigen::Matrix<Type, 3, 1> pointVector{xd, yd, zd};
    Point3D<Type> PointProjection{pointVector};
    _point = PointProjection;
}

template <typename Type>
void PointAndLine<Type>::GetDistance(Type& distance)
{
    Point3D<Type> pointProjection;
    GetProjection(pointProjection);
    DoublePoints<Type> dp{pointProjection,point};
    dp.GetDistance(distance);
}

/**** pointAndPlane ****/
template <typename Type>
PointAndPlane<Type>::PointAndPlane() {}

template <typename Type>
PointAndPlane<Type>::PointAndPlane(const Point3D<Type>& _point, const Plane3D<Type>& _plane) : 
    point(_point), plane(_plane) {}

template <typename Type>
PointAndPlane<Type>::PointAndPlane(const PointAndPlane& _pointAndPlane)
{
    point = _pointAndPlane.point;
    plane = _pointAndPlane.plane;
}

template <typename Type>
PointAndPlane<Type>::PointAndPlane(PointAndPlane&& _pointAndPlane)
{
    point = _pointAndPlane.point;
    plane = _pointAndPlane.plane;
}

template <typename Type>
PointAndPlane<Type>::~PointAndPlane() {}

template <typename Type>
bool PointAndPlane<Type>::IsIncluded()
{
    Eigen::Matrix<Type, 4, 1> pointHomogeneous;
    pointHomogeneous << point.x, point.y, point.z, 1.;
    Type dotProductValue = std::fabs(pointHomogeneous.dot(plane.plane));
    if(dotProductValue < INFINITESMAL)
        return true;
    else
        return false;
}

template <typename Type>
PointAndPlane<Type>& PointAndPlane<Type>::operator=(const PointAndPlane<Type>& _pointAndPlane)
{
    if(this == &_pointAndPlane)
        return *this;

    point = _pointAndPlane.point;
    plane = _pointAndPlane.plane;

    return *this;
}

template <typename Type>
void PointAndPlane<Type>::GetProjection(Point3D<Type>& _point)
{
    Eigen::Matrix<Type, 3, 1> planeNormVector{plane.a, plane.b, plane.c};
    auto mol = - (planeNormVector.dot(point.point) + plane.d);
    auto den = planeNormVector.squaredNorm();
    auto t = mol/den;

    Eigen::Matrix<Type, 3, 1> pointVector = point.point + t * planeNormVector;
    Point3D<Type> pointTmp{pointVector};
    _point = pointTmp;
}

template <typename Type>
void PointAndPlane<Type>::GetDistance(Type& distance)
{
    Eigen::Matrix<Type, 4, 1> pointHomogeneous{point.x, point.y, point.z, 1};
    auto mol = std::fabs(pointHomogeneous.dot(plane.plane));

    Eigen::Matrix<Type, 3, 1> normalVector{plane.a, plane.b, plane.c};
    auto den = normalVector.norm();

    distance = mol/den;
}

/**** LineAndPlane ****/
template <typename Type>
LineAndPlane<Type>::LineAndPlane() {}

template <typename Type>
LineAndPlane<Type>::LineAndPlane(const Line3D<Type>& _line, const Plane3D<Type>& _plane) : 
    line(_line), plane(_plane) {}

template <typename Type>
LineAndPlane<Type>::LineAndPlane(const LineAndPlane<Type>& _lineAndPlane)
{
    line = _lineAndPlane.line;
    plane = _lineAndPlane.plane;
}

template <typename Type>
LineAndPlane<Type>::LineAndPlane(LineAndPlane&& _lineAndPlane)
{
    line = _lineAndPlane.line;
    plane = _lineAndPlane.plane;
}

template <typename Type>
LineAndPlane<Type>::~LineAndPlane() {}

template <typename Type>
LineAndPlane<Type>& LineAndPlane<Type>::operator=(const LineAndPlane<Type>& _lineAndPlane)
{
    if(this == & _lineAndPlane)
        return *this;

    line = _lineAndPlane.line;
    plane = _lineAndPlane.plane;

    return *this;
}

template <typename Type>
bool LineAndPlane<Type>::IsParallel()
{
    auto lineVector = line.vector.vector;
    Eigen::Matrix<Type, 3, 1> planeNormalVector{plane.a, plane.b, plane.c};
    auto dotProjectValue = std::fabs(lineVector.dot(planeNormalVector));
    if(dotProjectValue < INFINITESMAL)
        return true;
    else
        return false;
}

template <typename Type>
bool LineAndPlane<Type>::IsVertical()
{
    auto lineVector = line.vector.vector;
    Eigen::Matrix<Type, 3, 1> planeNormalVector{plane.a, plane.b, plane.c};
    auto crossProjectValue = lineVector.cross(planeNormalVector).norm();
    if(crossProjectValue < INFINITESMAL)
        return true;
    else
        return false;
}

template <typename Type>
bool LineAndPlane<Type>::IsIncluded()
{
    Point3D<Type> point1{line.point.point};
    Eigen::Matrix<Type, 3, 1> pointVector;
    pointVector << line.point.x + line.vector.x, line.point.y + line.vector.y, line.point.z + line.vector.z;
    Point3D<Type> point2{pointVector};

    PointAndPlane<Type> pap1{point1, plane};
    PointAndPlane<Type> pap2{point2, plane};

    if( pap1.IsIncluded() && pap2.IsIncluded() )
        return true;
    else
        return false;
}

template <typename Type>
void LineAndPlane<Type>::GetAngle(Type& angle)
{
    Eigen::Matrix<Type, 3, 1> planeNormalVector{plane.a, plane.b, plane.c};
    auto cosT = fabs(planeNormalVector.dot(line.vector.vector)/planeNormalVector.norm()/line.vector.vector.norm());
    angle = M_PI/2 - std::acos(cosT);
}

template <typename Type>
void LineAndPlane<Type>::GetProjection(Line3D<Type>& _line)
{
    auto pointVector1 = line.point.point;
    auto pointVector2 = pointVector1 + line.vector.vector;
    Point3D<Type> point1{pointVector1};
    Point3D<Type> point2{pointVector2};
    PointAndPlane<Type> pap1(point1,plane);
    PointAndPlane<Type> pap2(point2,plane);
    Point3D<Type> pointProject1;
    Point3D<Type> pointProject2;
    pap1.GetProjection(pointProject1);
    pap2.GetProjection(pointProject2);

    auto pointToPointVector = pointProject2.point - pointProject1.point;
    Vector3D<Type> vectorTmp{pointToPointVector};
    Line3D<Type> lineTmp(pointProject1,vectorTmp);
    _line = lineTmp;
}

template <typename Type>
void LineAndPlane<Type>::GetIntersectionPoint(Point3D<Type>& point)
{
    if(IsParallel())
    {
        std::string error = "Error in GetIntersectionPoint(): line and plane are parallel.";
        throw std::runtime_error(error);
    }

    auto linePoint = line.point.point;
    auto lineVector = line.vector.vector;

    Eigen::Matrix<Type, 3, 1> planeNormVector{plane.a, plane.b, plane.c};

    Eigen::Matrix<Type, 3, 1> planePoint;
    if( std::fabs(plane.a) > INFINITESMAL )
    {
        Type x = -plane.d / plane.a;
        planePoint << x,0,0;
    }
    else if( std::fabs(plane.b) > INFINITESMAL)
    {
        Type y = -plane.d / plane.b;
        planePoint << 0,y,0;
    }
    else
    {
        Type z = -plane.d / plane.c;
        planePoint << 0,0,z;
    }

    auto pointToPointVector = planePoint - linePoint;
    auto mol = planeNormVector.dot(pointToPointVector);
    auto den = planeNormVector.dot(lineVector);

    auto t = mol/den;

    auto pointVector = linePoint + t * lineVector;
    Point3D<Type> pointTmp{pointVector};
    point = pointTmp;
}

}//namespace am

#endif //_SPACE_ANALYTIC_GEOMETRY_H