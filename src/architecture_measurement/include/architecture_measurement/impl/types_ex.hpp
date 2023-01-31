
#pragma once
#ifndef _TYPES_H
#define _TYPES_H

#include <cstdlib>

#include "Eigen/Core"

/**
 * @brief namespace architecture measurement. 
 * 
 */
namespace am 
{

/**
 * @brief 
 * 
 * @tparam Type 
 */
template <typename Type>
class Point3D
{
    public:
        Type x;/**< x coordinate of point */
        Type y;/**< y coordinate of point */
        Type z;/**< z coordinate of point */
        Eigen::Matrix<Type, 3, 1> point;/**< point represented by eigen vector*/

    public:
        /**
         * @brief Construct a new Point 3 D object
         * 
         */
        Point3D();

        /**
         * @brief Construct a new Point 3 D object
         * 
         * @param _x 
         * @param _y 
         * @param _z 
         */
        Point3D(const Type& _x,const Type& _y,const Type& _z);

        /**
         * @brief Construct a new Point 3d object
         * 
         * @param _point point represented by eigen vector
         */
        Point3D(const Eigen::Matrix<Type, 3, 1>& _point);

        /**
         * @brief Construct a new Point 3d object
         * 
         * @param _point a Point object
         */
        Point3D(const Point3D<Type>& _point); 

        /**
         * @brief Construct a new Point 3 D object
         * 
         * @param _point 
         */
        Point3D(Point3D<Type>&& _point);

        /**
         * @brief Destroy the Point 3d object
         * 
         */
        virtual ~Point3D();

        /**
         * @brief assignment operator
         * 
         * @param _point a Point object
         * @return Point3D& a Point3D object
         */
        Point3D<Type>& operator=(const Point3D<Type>& _point);
};

/**
 * @brief 
 * 
 * @tparam Type 
 */
template <typename Type>
class Vector3D
{
    public:
        Type x;/**< x coordinate of vector */
        Type y;/**< y coordinate of vector */
        Type z;/**< z coordinate of vector */
        Eigen::Matrix<Type, 3, 1> vector;/**< vcetor represented by eigen vector*/
    
    public:

        /**
         * @brief Construct a new Vector 3 D object
         * 
         */
        Vector3D();

        /**
         * @brief Construct a new Vector 3d object
         * 
         * @param _x x coordinate of vector
         * @param _y y coordinate of vector
         * @param _z z coordinate of vector
         */
        Vector3D(const Type& _x, const Type& _y, const Type& _z);

        /**
         * @brief Construct a new Vector 3d object
         * 
         * @param _vector vector represented by eigen vector
         */
        Vector3D(const Eigen::Matrix<Type, 3, 1>& _vector);

        /**
         * @brief Construct a new Vector 3d object
         * 
         * @param _vector a Vcetor object
         */
        Vector3D(const Vector3D<Type>& _vector);

        /**
         * @brief Construct a new Vector 3 D object
         * 
         * @param _vector 
         */
        Vector3D(Vector3D&& _vector);

        /**
         * @brief Destroy the Vector 3d object
         * 
         */
        virtual ~Vector3D();

        /**
         * @brief Assignment operator
         * 
         * @param _vector a Matrix<Type, 3, 1> object
         * @return Matrix<Type, 3, 1>& a Matrix<Type, 3, 1> object
         */
        Vector3D<Type>& operator=(const Vector3D<Type>& _vector);

        /**
         * @brief 
         * 
         */
        void Normalization();
};

/**
 * @brief 
 * 
 * @tparam Type 
 */
template <typename Type>
class Line3D
{
    public:
        Point3D<Type> point;/**< point on line */
        Vector3D<Type> vector;/**< line direation vector */
    public:
        /**
         * @brief Construct a new Line 3 D object
         * 
         */
        Line3D();

        /**
         * @brief Construct a new Line 3d object
         * 
         * @param _point a Point2d object
         * @param _vector a Matrix<Type, 3, 1> object
         */
        Line3D(const Point3D<Type>& _point, const Vector3D<Type>& _vector);

        /**
         * @brief Construct a new Line 3d object
         * 
         * @param _line a Line3D object
         */
        Line3D(const Line3D<Type>& _line);

        /**
         * @brief Construct a new Line 3 D object
         * 
         * @param _line 
         */
        Line3D(Line3D<Type>&& _line);

        /**
         * @brief Destroy the Line 3d object
         * 
         */
        ~Line3D();

        /**
         * @brief Assignment operator
         * 
         * @param _line a Line3D object
         * @return Line3D& a Line3D object
         */
        Line3D& operator=(const Line3D<Type>& _line);
};

/**
 * @brief 
 * 
 * @tparam Type 
 */
template <typename Type>
class Plane3D
{
    public:
        Type a;/**< plane coeffcients -- a*/
        Type b;/**< plane coeffcients -- b*/
        Type c;/**< plane coeffcients -- c*/
        Type d;/**< plane coeffcients -- d*/
        Eigen::Matrix<Type, 4, 1> plane;/**< plane represented by eigen vector */

    public:

        /**
         * @brief Construct a new Plane 3 D object
         * 
         */
        Plane3D();

        /**
         * @brief Construct a new Plane 3d object
         * 
         * @param _a plane coeffcients -- a
         * @param _b plane coeffcients -- b
         * @param _c plane coeffcients -- c
         * @param _d plane coeffcients -- d
         */
        Plane3D(const Type& _a, const Type& _b, const Type& _c, const Type& _d);

        /**
         * @brief Construct a new Plane 3d object
         * 
         * @param _plane 
         */
        Plane3D(const Eigen::Matrix<Type, 4, 1>& _plane);

        /**
         * @brief Construct a new Plane 3d object
         * 
         * @param _plane a Plane3D object
         */
        Plane3D(const Plane3D<Type>& _plane);

        /**
         * @brief Construct a new plane3 D object
         * 
         * @param _plane 
         */
        Plane3D(Plane3D<Type>&& _plane);  

        /**
         * @brief Destroy the Plane 3d object
         * 
         */
        ~Plane3D();

        /**
         * @brief Assignment operator
         * 
         * @param _plane a Plane3D object
         * @return Plane3D& a Plane3D object
         */
        Plane3D<Type>& operator=(const Plane3D<Type>& _plane);
};

/** Point3D **/
template <typename Type>
Point3D<Type>::Point3D() {}

template <typename Type>
Point3D<Type>::Point3D(const Type& _x,const Type& _y,const Type& _z) : x(_x),y(_y),z(_z)
{
    point << x, y, z;
}

template <typename Type>
Point3D<Type>::Point3D(const Eigen::Matrix<Type, 3, 1>& _point) :  point(_point)
{
    x = point(0);
    y = point(1);
    z = point(2);
}

template <typename Type>
Point3D<Type>::Point3D(const Point3D<Type>& _point)
{
    x = _point.x;
    y = _point.y;
    z = _point.z;
    point = _point.point;
}

template <typename Type>
Point3D<Type>::Point3D(Point3D<Type>&& _point)
{
    x = _point.x;
    y = _point.y;
    z = _point.z;
    point = _point.point;
}

template <typename Type>
Point3D<Type>::~Point3D() {}

template <typename Type>
Point3D<Type>& Point3D<Type>::operator=(const Point3D<Type>& _point)
{
    if(this == &_point)
        return *this;

    x = _point.x;
    y = _point.y;
    z = _point.z;
    point = _point.point;

    return *this;
}

/** Vector3D **/
template <typename Type>
Vector3D<Type>::Vector3D() {}

template <typename Type>
Vector3D<Type>::Vector3D(const Type& _x,const Type& _y,const Type& _z) : x(_x), y(_y), z(_z) 
{
    vector << x, y, z;
}

template <typename Type>
Vector3D<Type>::Vector3D(const Eigen::Matrix<Type, 3, 1>& _vector) : vector(_vector)
{
    x = vector(0);
    y = vector(1);
    z = vector(2);
}

template <typename Type>
Vector3D<Type>::Vector3D(const Vector3D<Type>& _vector)
{
    x = _vector.x;
    y = _vector.y;
    z = _vector.z;
    vector = _vector.vector;
}

template <typename Type>
Vector3D<Type>::Vector3D(Vector3D<Type>&& _vector)
{
    x = _vector.x;
    y = _vector.y;
    z = _vector.z;
    vector = _vector.vector;
}

template <typename Type>
Vector3D<Type>::~Vector3D() {}

template <typename Type>
Vector3D<Type>& Vector3D<Type>::operator=(const Vector3D<Type>& _vector)
{
    if(this == &_vector)
        return *this;

    x = _vector.x;
    y = _vector.y;
    z = _vector.z;
    vector = _vector.vector;

    return *this;
}

template <typename Type>
void Vector3D<Type>::Normalization()
{
    vector = vector/vector.norm();
    x = vector(0);
    y = vector(1);
    z = vector(2);
}

/** Line3D **/
template <typename Type>
Line3D<Type>::Line3D(){}

template <typename Type>
Line3D<Type>::Line3D(const Point3D<Type>& _point, const Vector3D<Type>& _vector) : point(_point), vector(_vector) 
{
    vector.Normalization();
}

template <typename Type>
Line3D<Type>::Line3D(const Line3D<Type>& _line)
{
    point = _line.point;
    vector = _line.vector;
}

template <typename Type>
Line3D<Type>::Line3D(Line3D<Type>&& _line)
{
    point = _line.point;
    vector = _line.vector;
}

template <typename Type>
Line3D<Type>::~Line3D() {}

template <typename Type>
Line3D<Type>& Line3D<Type>::operator=(const Line3D<Type>& _line)
{
    if(this == &_line)
        return *this;

    point = _line.point;
    vector = _line.vector;

    return *this;
}

/** Plane3D **/
template <typename Type> 
Plane3D<Type>::Plane3D() {}

template <typename Type>
Plane3D<Type>::Plane3D(const Type& _a, const Type& _b, const Type& _c, const Type& _d) : a(_a), b(_b), c(_c), d(_d)
{
    Eigen::Matrix<Type, 3, 1> normVector{a, b, c};
    Eigen::Matrix<Type, 3, 1> normVectorNormalization = normVector/normVector.norm();
    a = normVectorNormalization(0);
    b = normVectorNormalization(1);
    c = normVectorNormalization(2);
    d = d/normVector.norm();
    plane << a, b, c, d;
}

template <typename Type>
Plane3D<Type>::Plane3D(const Eigen::Matrix<Type, 4, 1>& _plane) : plane(_plane)
{
    Eigen::Matrix<Type, 3, 1> normVector{plane(0), plane(1), plane(2)};
    Eigen::Matrix<Type, 3, 1> normVectorNormalization = normVector/normVector.norm();
    a = normVectorNormalization(0);
    b = normVectorNormalization(1);
    c = normVectorNormalization(2);
    d = plane(3)/normVector.norm();
    plane << a, b, c, d;
}

template <typename Type>
Plane3D<Type>::Plane3D(const Plane3D<Type>& _plane)
{
    a = _plane.a;
    b = _plane.b;
    c = _plane.c;
    d = _plane.d;
    plane = _plane.plane;
}

template <typename Type>
Plane3D<Type>::Plane3D(Plane3D<Type>&& _plane)
{
    a = _plane.a;
    b = _plane.b;
    c = _plane.c;
    d = _plane.d;
    plane = _plane.plane;
}

template <typename Type>
Plane3D<Type>::~Plane3D() {}

template <typename Type>
Plane3D<Type>& Plane3D<Type>::operator=(const Plane3D<Type>& _plane)
{
    if(this == &_plane)
        return *this;

    a = _plane.a;
    b = _plane.b;
    c = _plane.c;
    d = _plane.d;
    plane = _plane.plane;

    return *this;
}

/**
 * @brief Calculate vertical plane of line
 * 
 * @param line a Line3D object
 * @return Plane3D a Plane3D object
 */
template <typename Type>
Plane3D<Type> ConvertToPlane(const Line3D<Type>& _line)
{
    Eigen::Matrix<Type, 3, 1> vector{_line.vector.x, _line.vector.y, _line.vector.z};
    Eigen::Matrix<Type, 3, 1> point{_line.point.x, _line.point.y, _line.point.z};

    Plane3D<Type> plane;
    plane.a = _line.vector.x;
    plane.b = _line.vector.y;
    plane.c = _line.vector.z;
    plane.d = -vector.dot(point);
    plane.plane = Eigen::Matrix<Type, 4, 1>(plane.a, plane.b, plane.c, plane.d);

    return plane;
}

/**
 * @brief calculate vertical line of plane
 * 
 * @param plane a Plane3D object
 * @return Line3D a Line3D object
 */
template <typename Type>
Line3D<Type> ConvertToLine(const Plane3D<Type>& _plane)
{
    Eigen::Matrix<Type, 3, 1> vector{_plane.a, _plane.b, _plane.c};
    Eigen::Matrix<Type, 3, 1> point{0, 0, 0};
    Point3D<Type> point3D{point};
    Vector3D<Type> vector3D{vector};
    Line3D<Type> line3D{point, vector};
    return line3D;
}

}//namespace am

#endif //_TYPES_H

